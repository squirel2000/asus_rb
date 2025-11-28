import serial
import numpy as np
import argparse
import time
import threading
from datetime import datetime
from collections import deque

# Head control for the robot using serial communication, and the protocol defined in:
# https://asus.sharepoint.com/:w:/r/sites/ZenGimbal_Ver2.0/_layouts/15/Doc.aspx?sourcedoc=%7B563FB2AC-D8AA-4B42-B4BB-808C06ECD9F9%7D&file=TOBY%20communications%20protocol_beta(versionb0.91).docx

# Constants
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200

# Yaw and pitch limits (in degrees)
YAW_MIN, YAW_MAX = -45.0, 45.0
PITCH_MIN, PITCH_MAX = -15.0, 55.0

# Feedback tags
FEEDBACK_TAG_BASIC_SENSOR_DATA = 0x01
FEEDBACK_TAG_HARDWARE_VERSION = 0x10
FEEDBACK_TAG_FIRMWARE_VERSION = 0x11
FEEDBACK_TAG_HW_DROP_IR = 0x12
FEEDBACK_TAG_WHEEL_ENCODER = 0x20
FEEDBACK_TAG_NECK_ENCODER = 0x21
FEEDBACK_TAG_ODOMETRY = 0x22
FEEDBACK_TAG_AVOID_TARGET = 0x23
FEEDBACK_TAG_ERROR_CODE = 0xE0

# Command ID for requesting extra information
REQUEST_EXTRA_INFO_CMD = 0x09
REQUEST_FEEDBACK_CMD = 0x08
# Feedback Request Flags
REQUEST_FLAG_MOTOR_CURRENT = (1 << 8)
# Current Protection
OCP_NECK_YAW_MASK   = (1 << 0)
OCP_NECK_PITCH_MASK = (1 << 1)
OLP_NECK_YAW_MASK   = (1 << 4)
OLP_NECK_PITCH_MASK = (1 << 5)
# Over Temperature
TEMP_NECK_YAW_MASK   = (1 << 0)
TEMP_NECK_PITCH_MASK = (1 << 1)
# Neck Status
NECK_YAW_FW_ERR_MASK   = (1 << 4)
NECK_PITCH_FW_ERR_MASK = (1 << 5)
NECK_YAW_HW_ERR_MASK   = (1 << 6)
NECK_PITCH_HW_ERR_MASK = (1 << 7)

def log(message):
    """Helper function to print messages with a timestamp."""
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    print(f"[{timestamp}] {message}")


class HeadController:
    def __init__(self, logger=None):
        self.logger = logger

        try:
            self.serial_port = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01) # Small timeout for non-blocking read
        except serial.SerialException as e:
            self._log(f"Error: Could not open serial port: {e}", 'error')
            self.serial_port = None

        self.smoothed_yaw = 0.0
        self.smoothed_pitch = 0.0

        self._running = False
        self._read_thread = None
        self._read_buffer = bytearray()
        self._feedback_data = {}
        self._feedback_data_lock = threading.Lock()
        self._version_info_ready = threading.Event()
        self._last_version_response_raw = None # Store the raw packet for debugging

        # Error states from Basic Sensor Data
        self.error_states = {
            'OCP_NECK_YAW': False, 'OCP_NECK_PITCH': False, 'OLP_NECK_YAW': False,
            'OLP_NECK_PITCH': False, 'TEMP_NECK_YAW': False, 'TEMP_NECK_PITCH': False,
            'NECK_YAW_FW_ERR': False, 'NECK_PITCH_FW_ERR': False,
            'NECK_YAW_HW_ERR': False, 'NECK_PITCH_HW_ERR': False,
        }
        self.last_error_log_time = {} # Used to throttle error logging

        # Neck angle & velocity tracking
        self.current_neck_yaw_deg = 0.0
        self.current_neck_pitch_deg = 0.0

        self.last_neck_update_time = None
        self.last_neck_yaw_deg = None
        self.last_neck_pitch_deg = None
        self.current_neck_yaw_vel_dps = 0.0
        self.current_neck_pitch_vel_dps = 0.0

        # --- Velocity Smoothing ---
        # 1st order low-pass filter (EMA). Alpha is the smoothing factor.
        # A smaller alpha means more smoothing but more lag.
        self.velocity_filter_alpha = 0.4

        # For sampling time analysis
        self.dt_history = deque(maxlen=200) # Store last 200 dt values

        # Motor current tracking
        self.current_neck_yaw_ma = 0.0
        self.current_neck_pitch_ma = 0.0

    def _log(self, msg, level='info'):
        if self.logger:
            if level == 'info':
                self.logger.info(msg)
            elif level == 'warn':
                self.logger.warn(msg)
            elif level == 'error':
                self.logger.error(msg)
            else:
                self.logger.info(msg)
        else:
            # Fallback to print if no logger is provided
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            print(f"[{timestamp}] [{level.upper()}] {msg}")

    # -------------------------------------------------------------
    # Listener thread
    # -------------------------------------------------------------
    def start_listening(self):
        if self.serial_port and self.serial_port.is_open and not self._running:
            self._running = True
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            self._log("Started listening for feedback packets.")
        elif self._running:
            self._log("Listener already running.")
        else:
            self._log("Warning: Serial port not available to start listener.", 'warn')

    def stop_listening(self):
        if self._running:
            self._running = False
            if self._read_thread:
                self._read_thread.join(timeout=1)
            self._log("Stopped listening for feedback packets.")

    def _read_loop(self):
        while self._running:
            try:
                bytes_to_read = self.serial_port.in_waiting
                if bytes_to_read > 0:
                    data = self.serial_port.read(bytes_to_read)
                    self._read_buffer.extend(data)
                
                self._process_buffer_for_packets()

                if bytes_to_read == 0:
                    time.sleep(0.005)

            except serial.SerialException as e:
                self._log(f"FATAL: Serial communication error: {e}.", 'error')
                self._running = False
            except Exception as e:
                self._log(f"Error in read loop: {e}", 'error')
                time.sleep(0.05)

    # -------------------------------------------------------------
    # Packet parsing
    # -------------------------------------------------------------
    def _process_buffer_for_packets(self):
        while len(self._read_buffer) >= 4:
            start_index = self._read_buffer.find(b'\xaa\x55')

            if start_index == -1:
                self._read_buffer.clear()
                break

            if start_index > 0:
                self._read_buffer = self._read_buffer[start_index:]

            if len(self._read_buffer) < 3:
                break

            payload_len = self._read_buffer[2]
            packet_len = payload_len + 4

            if len(self._read_buffer) < packet_len:
                break

            packet = self._read_buffer[:packet_len]
            self._read_buffer = self._read_buffer[packet_len:]

            checksum = 0
            for b in packet[2:-1]:
                checksum ^= b

            if checksum != packet[-1]:
                self._log(f"Warning: Checksum mismatch for packet: {packet.hex(' ')}", 'warn')
                continue

            self._parse_feedback_payload(packet)

    # -------------------------------------------------------------
    # Utility
    # -------------------------------------------------------------
    def _build_packet(self, payload):
        packet = bytearray([0xAA, 0x55, len(payload)]) + payload
        checksum = 0
        for b in packet[2:]:
            checksum ^= b
        packet.append(checksum)
        return packet

    # --------------------------------------------------------------------
    # NEW — Neck velocity estimation (computed from encoder feedback)
    # --------------------------------------------------------------------
    def _update_neck_velocity(self, yaw_deg, pitch_deg):
        now = time.time()

        if self.last_neck_update_time is not None:
            dt = now - self.last_neck_update_time
            if dt > 0:
                self.dt_history.append(dt)
                dy = yaw_deg - self.last_neck_yaw_deg
                dp = pitch_deg - self.last_neck_pitch_deg

                # Calculate instantaneous velocity
                inst_yaw_vel = dy / dt
                inst_pitch_vel = dp / dt

                # Apply the low-pass filter (Exponential Moving Average)
                alpha = self.velocity_filter_alpha
                self.current_neck_yaw_vel_dps = alpha * inst_yaw_vel + (1 - alpha) * self.current_neck_yaw_vel_dps
                self.current_neck_pitch_vel_dps = alpha * inst_pitch_vel + (1 - alpha) * self.current_neck_pitch_vel_dps

        self.last_neck_update_time = now
        self.last_neck_yaw_deg = yaw_deg
        self.last_neck_pitch_deg = pitch_deg

    def _log_error_throttle(self, error_key, message):
        """Logs an error message for a given key, but not more than once every 10s."""
        now = time.time()
        last_log_time = self.last_error_log_time.get(error_key, 0)
        if now - last_log_time > 10.0:
            self._log(message, 'error')
            self.last_error_log_time[error_key] = now

    # -------------------------------------------------------------
    # Parse incoming sensor packets
    # -------------------------------------------------------------
    def _parse_feedback_payload(self, packet):
        main_payload = packet[3:-1]

        i = 0
        found_version = False

        with self._feedback_data_lock:
            while i < len(main_payload):
                tag = main_payload[i]
                if i + 1 >= len(main_payload):
                    break

                block_len = main_payload[i + 1]
                if i + 2 + block_len > len(main_payload):
                    break

                value = main_payload[i + 2: i + 2 + block_len]

                # ---------------------------------------------------------
                # Basic Sensor Data - includes multiple status bytes
                # ---------------------------------------------------------
                if tag == FEEDBACK_TAG_BASIC_SENSOR_DATA and block_len >= 9:
                    # Per protocol doc, data is:
                    # timestamp (4), ir_drop (1), sonar (1), current (1), temp (1), neck (1), ...
                    current_prot_byte = value[6]
                    over_temp_byte = value[7]
                    neck_status_byte = value[8]

                    # Check and log Current Protection errors
                    self.error_states['OCP_NECK_YAW'] = (current_prot_byte & OCP_NECK_YAW_MASK) != 0
                    if self.error_states['OCP_NECK_YAW']:
                        self._log_error_throttle('OCP_NECK_YAW', "MCU Error: Over Current Protection on Neck Yaw.")
                    
                    self.error_states['OCP_NECK_PITCH'] = (current_prot_byte & OCP_NECK_PITCH_MASK) != 0
                    if self.error_states['OCP_NECK_PITCH']:
                        self._log_error_throttle('OCP_NECK_PITCH', "MCU Error: Over Current Protection on Neck Pitch.")

                    self.error_states['OLP_NECK_YAW'] = (current_prot_byte & OLP_NECK_YAW_MASK) != 0
                    if self.error_states['OLP_NECK_YAW']:
                        self._log_error_throttle('OLP_NECK_YAW', "MCU Error: Over Load Protection on Neck Yaw.")

                    self.error_states['OLP_NECK_PITCH'] = (current_prot_byte & OLP_NECK_PITCH_MASK) != 0
                    if self.error_states['OLP_NECK_PITCH']:
                        self._log_error_throttle('OLP_NECK_PITCH', "MCU Error: Over Load Protection on Neck Pitch.")

                    # Check and log Over Temperature errors
                    self.error_states['TEMP_NECK_YAW'] = (over_temp_byte & TEMP_NECK_YAW_MASK) != 0
                    if self.error_states['TEMP_NECK_YAW']:
                        self._log_error_throttle('TEMP_NECK_YAW', "MCU Error: Over Temperature on Neck Yaw.")
                    
                    self.error_states['TEMP_NECK_PITCH'] = (over_temp_byte & TEMP_NECK_PITCH_MASK) != 0
                    if self.error_states['TEMP_NECK_PITCH']:
                        self._log_error_throttle('TEMP_NECK_PITCH', "MCU Error: Over Temperature on Neck Pitch.")

                    # Check and log Neck Status errors
                    self.error_states['NECK_YAW_FW_ERR'] = (neck_status_byte & NECK_YAW_FW_ERR_MASK) != 0
                    if self.error_states['NECK_YAW_FW_ERR']:
                        self._log_error_throttle('NECK_YAW_FW_ERR', "MCU Error: Neck Yaw Firmware Error.")

                    self.error_states['NECK_PITCH_FW_ERR'] = (neck_status_byte & NECK_PITCH_FW_ERR_MASK) != 0
                    if self.error_states['NECK_PITCH_FW_ERR']:
                        self._log_error_throttle('NECK_PITCH_FW_ERR', "MCU Error: Neck Pitch Firmware Error.")

                    self.error_states['NECK_YAW_HW_ERR'] = (neck_status_byte & NECK_YAW_HW_ERR_MASK) != 0
                    if self.error_states['NECK_YAW_HW_ERR']:
                        self._log_error_throttle('NECK_YAW_HW_ERR', "MCU Error: Neck Yaw Hardware Error.")

                    self.error_states['NECK_PITCH_HW_ERR'] = (neck_status_byte & NECK_PITCH_HW_ERR_MASK) != 0
                    if self.error_states['NECK_PITCH_HW_ERR']:
                        self._log_error_throttle('NECK_PITCH_HW_ERR', "MCU Error: Neck Pitch Hardware Error.")

                    # Decode motor current if available (bytes 9-12)
                    if block_len >= 13:
                        self.current_neck_yaw_ma = int.from_bytes(value[9:11], 'little', signed=True)
                        self.current_neck_pitch_ma = int.from_bytes(value[11:13], 'little', signed=True)
                        self._feedback_data["Motor Current"] = {
                            "yaw_mA": self.current_neck_yaw_ma,
                            "pitch_mA": self.current_neck_pitch_ma
                        }
                # ---------------------------------------------------------
                # Neck Encoder — includes (yaw,pitch) in 0.1 deg units
                # ---------------------------------------------------------
                elif tag == FEEDBACK_TAG_NECK_ENCODER and block_len == 4:
                    yaw_raw = int.from_bytes(value[0:2], 'little', signed=True)
                    pitch_raw = int.from_bytes(value[2:4], 'little', signed=True)

                    yaw_deg = yaw_raw / 10.0
                    pitch_deg = pitch_raw / 10.0

                    self.current_neck_yaw_deg = yaw_deg
                    self.current_neck_pitch_deg = pitch_deg

                    # NEW — compute velocity
                    self._update_neck_velocity(yaw_deg, pitch_deg)

                    self._feedback_data["Neck Encoder"] = {
                        "yaw_deg": yaw_deg,
                        "pitch_deg": pitch_deg,
                        "yaw_vel_dps": self.current_neck_yaw_vel_dps,
                        "pitch_vel_dps": self.current_neck_pitch_vel_dps
                    }

                # ---------------------------------------------------------
                # Hardware version, firmware, other feedback
                # ---------------------------------------------------------
                elif tag == FEEDBACK_TAG_HARDWARE_VERSION and block_len == 4:
                    try:
                        name = value[2:].decode('ascii', errors='ignore')
                        self._feedback_data["Hardware Version"] = \
                            f"{name} v{value[1]}.{value[0]}"
                        found_version = True
                    except:
                        self._feedback_data[f"TAG_0x{tag:02x}"] = value.hex(' ')

                elif tag == FEEDBACK_TAG_FIRMWARE_VERSION and block_len == 4:
                    try:
                        year = 2000 + value[1]
                        month = value[2]
                        day = value[3]
                        self._feedback_data["Firmware Build Date"] = \
                            f"{year}-{month:02d}-{day:02d}"
                        found_version = True
                    except:
                        self._feedback_data[f"TAG_0x{tag:02x}"] = value.hex(' ')

                else:
                    self._feedback_data[f"TAG_0x{tag:02x}"] = value.hex(' ')

                i += 2 + block_len

            if found_version:
                self._last_version_response_raw = packet
                self._version_info_ready.set()

    # -------------------------------------------------------------
    # Build commands
    # -------------------------------------------------------------
    def build_neck_position_command(self, yaw_deg=0.0, pitch_deg=0.0, duration_ms=50):
        yaw_val = int(yaw_deg * 10)
        pitch_val = int(pitch_deg * 10)

        def to_bytes_le(val, length=2):
            return val.to_bytes(length, 'little', signed=True)

        payload = bytearray([0x04, 0x08])
        payload += to_bytes_le(yaw_val)
        payload += to_bytes_le(pitch_val)
        payload += to_bytes_le(duration_ms)
        payload += to_bytes_le(duration_ms)
        return self._build_packet(payload)

    # --------------------------------------------------------------------
    # Neck Velocity Control (Command 0x03)
    # --------------------------------------------------------------------
    def build_neck_velocity_command(self, yaw_vel_dps=0.0, pitch_vel_dps=0.0,
                                    time_yaw_ms=200, time_pitch_ms=200):

        yaw_val = int(yaw_vel_dps * 10)      # convert deg/s → 0.1 deg/s
        pitch_val = int(pitch_vel_dps * 10)

        def to_bytes_le(val, length=2):
            return val.to_bytes(length, 'little', signed=True)

        payload = bytearray([0x03, 0x08])  # Command ID=03, payload length=8
        payload += to_bytes_le(yaw_val)
        payload += to_bytes_le(pitch_val)
        payload += to_bytes_le(time_yaw_ms, 2)
        payload += to_bytes_le(time_pitch_ms, 2)

        return self._build_packet(payload)

    # -------------------------------------------------------------
    # User API for Head Angle Control
    # -------------------------------------------------------------
    def control_head(self, yaw_deg, pitch_deg, duration_ms=300, logging=False):
        final_yaw_deg = max(YAW_MIN, min(YAW_MAX, yaw_deg))
        final_pitch_deg = max(PITCH_MIN, min(PITCH_MAX, pitch_deg))

        cmd = self.build_neck_position_command(final_yaw_deg, final_pitch_deg, duration_ms)

        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(cmd)
            except Exception as e:
                self._log(f"Error writing head control command: {e}", 'error')
                return

            if logging:
                self._log(f"Sent head position command: {cmd.hex(' ')}")
                self._log(f"  Target: yaw={final_yaw_deg:.2f}, pitch={final_pitch_deg:.2f}, duration={duration_ms}ms")
        else:
            self._log("Warning: Serial port not open to send head control command.", 'warn')

    # --------------------------------------------------------------------
    # User API for Velocity Control
    # --------------------------------------------------------------------
    def control_head_velocity(self, yaw_vel_dps, pitch_vel_dps,
                              duration_ms=300, logging=False):
        cmd = self.build_neck_velocity_command(
            yaw_vel_dps, pitch_vel_dps,
            duration_ms, duration_ms
        )

        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(cmd)
            except Exception as e:
                self._log(f"Error writing neck velocity command: {e}", 'error')
                return

            if logging:
                self._log(f"Sent neck velocity command: {cmd.hex(' ')}")
                self._log(f"  Target yaw_vel={yaw_vel_dps:.2f} deg/s, "
                          f"pitch_vel={pitch_vel_dps:.2f} deg/s, "
                          f"duration={duration_ms}ms")
        else:
            self._log("Warning: Serial port not open for velocity control.", 'warn')

    # -------------------------------------------------------------
    # Firmware version query
    # -------------------------------------------------------------
    def _build_version_request(self, enable=True):
        request_mask = 0x1F if enable else 0x00
        payload = bytearray([REQUEST_EXTRA_INFO_CMD, 0x01, request_mask])
        return self._build_packet(payload)

    def build_feedback_request(self, flags):
        """Builds a packet to request feedback data based on flags."""
        payload = bytearray([REQUEST_FEEDBACK_CMD, 0x04]) # 4 bytes for the flags
        payload += flags.to_bytes(4, 'little')
        return self._build_packet(payload)

    def get_all_feedback_data(self):
        with self._feedback_data_lock:
            return self._feedback_data.copy()

    def get_firmware_version(self, timeout=5):
        if not (self.serial_port and self.serial_port.is_open and self._running):
            return {"error": "Serial port not available or listener not running"}

        cmd = self._build_version_request(True)
        self._log(f"Sending version request: {cmd.hex(' ')}")
        self._version_info_ready.clear()

        try:
            self.serial_port.write(cmd)
        except Exception as e:
            msg = f"Error sending version request: {e}"
            self._log(msg, 'error')
            return {"error": msg}

        if not self._version_info_ready.wait(timeout=timeout):
            msg = f"Timeout waiting for version response."
            self._log(msg, 'warn')
            return {"error": msg}

        return self.get_all_feedback_data()

    def get_sampling_rate_stats(self):
        """
        Calculates and returns statistics about the neck encoder sampling rate.
        """
        with self._feedback_data_lock: # Lock to safely access dt_history
            if not self.dt_history:
                return None
            
            dt_samples = list(self.dt_history)

        avg_dt = np.mean(dt_samples)
        avg_hz = 1.0 / avg_dt if avg_dt > 0 else 0
        min_hz = 1.0 / np.max(dt_samples) if np.max(dt_samples) > 0 else 0
        max_hz = 1.0 / np.min(dt_samples) if np.min(dt_samples) > 0 else 0
        std_hz = np.std([1.0/dt for dt in dt_samples])
        return {"avg_hz": avg_hz, "min_hz": min_hz, "max_hz": max_hz, "std_hz": std_hz, "samples": len(dt_samples)}

    # -------------------------------------------------------------
    # Cleanup
    # -------------------------------------------------------------
    def destroy(self):
        self.stop_listening()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self._log("Serial port closed.")


# -------------------------------------------------------------------------
# Main entry point
# -------------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Control the robot's head and get firmware version.")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw angle in degrees for head control.")
    parser.add_argument("--pitch", type=float, default=0.0, help="Pitch angle in degrees for head control.")
    parser.add_argument("--duration", type=int, default=2000, help="Duration in milliseconds for head control.")
    parser.add_argument("--monitor-duration", type=int, default=10, help="Duration in seconds to monitor head position after command.")
    args = parser.parse_args()

    head_controller = HeadController()
    head_controller.start_listening()
    time.sleep(0.5)

    if head_controller.serial_port and head_controller.serial_port.is_open:
        log("\n--- Querying Firmware Version and other Extra Info ---")
        version_info = head_controller.get_firmware_version(timeout=3)
        
        if head_controller._last_version_response_raw:
            log(f"  Raw response data: {head_controller._last_version_response_raw.hex(' ')}")

        if version_info and "error" not in version_info:
            for key, value in sorted(version_info.items()):
                log(f"  {key}: {value}")
        else:
            log(f"  Could not retrieve or parse extra information: {version_info.get('error', 'Unknown error')}")
        log("-" * 60)

        log("\n--- Sending Head Control Command ---")
        head_controller.control_head(args.yaw, args.pitch, args.duration, logging=True)
        log("-" * 60)
        time.sleep(args.duration / 1000 + 1.0)  # 1.0s: Wait for the head to reach the position

        log(f"\n--- Monitoring Neck Positions for {args.monitor_duration} seconds (1 Hz) ---")
        for i in range(args.monitor_duration):
            if not head_controller._running:
                log("Connection to device lost. Stopping monitor.")
                break

            log(f"  Time {i+1}s: "
                f"Yaw={head_controller.current_neck_yaw_deg:.1f} deg, "
                f"Pitch={head_controller.current_neck_pitch_deg:.1f} deg, "
                f"Yaw_vel={head_controller.current_neck_yaw_vel_dps:.2f} deg/s, "
                f"Pitch_vel={head_controller.current_neck_pitch_vel_dps:.2f} deg/s")
            yaw_angle_deg = args.yaw + np.random.uniform(YAW_MIN, YAW_MAX)
            pitch_angle_deg = 20.0  # pitch_angle_deg = args.pitch + np.random.uniform(PITCH_MIN, PITCH_MAX)
            head_controller.control_head(yaw_angle_deg, pitch_angle_deg, args.duration, logging=True)
            time.sleep(1)
        
        log("\n--- Returning Head to Neutral Position (0.0 deg, 20.0 deg) ---")
        head_controller.control_head(0.0, 20.0, args.duration, logging=True)
        time.sleep(args.duration / 1000.0 + 0.5) # Wait for movement
        log("-" * 60)

        # --- Velocity Control Test ---
        log("\n--- Testing Velocity Control ---")
        # 1. Test Yaw Velocity
        test_yaw_vel = 10.0  # deg/s
        test_duration_ms = 1000  # ms
        log(f"Commanding yaw velocity of {test_yaw_vel} deg/s for {test_duration_ms/1000.0}s...")
        head_controller.control_head_velocity(yaw_vel_dps=test_yaw_vel, pitch_vel_dps=0.0, duration_ms=test_duration_ms, logging=True)

        # Monitor while the command is active
        for _ in range(10): # Monitor for 2 seconds at 5Hz
            log(f"  Monitoring: Yaw={head_controller.current_neck_yaw_deg:.1f}, Pitch={head_controller.current_neck_pitch_deg:.1f}, "
                f"YawVel={head_controller.current_neck_yaw_vel_dps:.2f} dps, PitchVel={head_controller.current_neck_pitch_vel_dps:.2f} dps")
            time.sleep(0.2)

        log("\n--- Returning Head to Neutral Position (0.0 deg, 20.0 deg) ---")
        head_controller.control_head(0.0, 20.0, args.duration, logging=True)
        time.sleep(args.duration / 1000.0 + 0.5) # Wait for movement

        # 2. Test Pitch Velocity
        test_pitch_vel = 10.0 # deg/s
        log(f"\nCommanding pitch velocity of {test_pitch_vel} deg/s for {test_duration_ms/1000.0}s...")
        head_controller.control_head_velocity(yaw_vel_dps=0.0, pitch_vel_dps=test_pitch_vel, duration_ms=test_duration_ms, logging=True)

        # Monitor while the command is active
        for _ in range(10): # Monitor for 2 seconds at 5Hz
            log(f"  Monitoring: Yaw={head_controller.current_neck_yaw_deg:.1f}, Pitch={head_controller.current_neck_pitch_deg:.1f}, "
                f"YawVel={head_controller.current_neck_yaw_vel_dps:.2f} dps, PitchVel={head_controller.current_neck_pitch_vel_dps:.2f} dps")
            time.sleep(0.2)
        log("-" * 60)

        # Return head to neutral position
        log("\n--- Returning Head to Neutral Position (0.0 deg, 20.0 deg) ---")
        head_controller.control_head(0.0, 20.0, args.duration, logging=True)
        time.sleep(args.duration / 1000.0 + 0.5) # Wait for movement

        # --- Report Sampling Rate ---
        log("\n--- MCU Sampling Rate Statistics ---")
        stats = head_controller.get_sampling_rate_stats()
        if stats:
            log(f"  Based on the last {stats['samples']} samples:")
            log(f"  Avg Rate: {stats['avg_hz']:.2f} Hz")
            log(f"  Min Rate: {stats['min_hz']:.2f} Hz, Max Rate: {stats['max_hz']:.2f} Hz, Std Dev: {stats['std_hz']:.2f} Hz")
        log("-" * 60)
        
        log("\n--- Keeping head at neutral position for 300 seconds and monitoring current ---")
        head_controller.control_head(0.0, 20.0, 1000, logging=True)
        time.sleep(1.5)  # Wait for head to settle

        start_time = time.time()
        loop_counter = 0
        while time.time() - start_time < 300:
            # Request motor current every loop
            current_request_cmd = head_controller.build_feedback_request(REQUEST_FLAG_MOTOR_CURRENT)
            if head_controller.serial_port and head_controller.serial_port.is_open:
                head_controller.serial_port.write(current_request_cmd)

            if loop_counter % 5 == 0:  # Log every 5 seconds
                log(f"  Time {int(time.time() - start_time)}s: "
                    f"Yaw={head_controller.current_neck_yaw_deg:.1f} deg, "
                    f"Pitch={head_controller.current_neck_pitch_deg:.1f} deg, "
                    f"Yaw Current={head_controller.current_neck_yaw_ma} mA, "
                    f"Pitch Current={head_controller.current_neck_pitch_ma} mA")

            time.sleep(1)
            loop_counter += 1

        head_controller.destroy()

    else:
        log("Exiting due to serial port not being available.")
