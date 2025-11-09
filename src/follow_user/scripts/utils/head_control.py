import serial
import math
import argparse
import time
import threading
from datetime import datetime

# Head control for the robot using serial communication, and the protocol defined in:
# https://asus.sharepoint.com/:w:/r/sites/ZenGimbal_Ver2.0/_layouts/15/Doc.aspx?sourcedoc=%7B563FB2AC-D8AA-4B42-B4BB-808C06ECD9F9%7D&file=TOBY%20communications%20protocol_beta(versionb0.91).docx&wdOrigin=TEAMS-WEB.p2p_ns.rwc&action=default&mobileredirect=true
#
# Usage: For example:
# python3 src/follow_user/scripts/utils/head_control.py --yaw 10.0 --pitch -5.0 --duration 1000 --monitor-duration 10

# Constants
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200
ALPHA = 0.1  # Smoothing factor for low-pass filter (applies to degrees)
# Yaw and pitch limits (in degrees)
YAW_MIN, YAW_MAX = -45.0, 45.0
PITCH_MIN, PITCH_MAX = -15.0, 55.0

# Define feedback tag mappings for better readability from toby.pdf
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

def log(message):
    """Helper function to print messages with a timestamp."""
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    print(f"[{timestamp}] {message}")

class HeadController:
    def __init__(self):
        try:
            self.serial_port = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01) # Small timeout for non-blocking read
        except serial.SerialException as e:
            log(f"Error: Could not open serial port: {e}")
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

        self.current_neck_yaw_deg = 0.0
        self.current_neck_pitch_deg = 0.0

    def start_listening(self):
        if self.serial_port and self.serial_port.is_open and not self._running:
            self._running = True
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            log("Started listening for feedback packets.")
        elif self._running:
            log("Listener already running.")
        else:
            log("Warning: Serial port not available to start listener.")

    def stop_listening(self):
        if self._running:
            self._running = False
            if self._read_thread:
                self._read_thread.join(timeout=2)
                if self._read_thread.is_alive():
                    log("Warning: Read thread did not terminate gracefully.")
            log("Stopped listening for feedback packets.")

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
                log(f"FATAL: Serial communication error: {e}. The device may have disconnected.")
                self._running = False # Stop the thread on serial error
            except Exception as e:
                log(f"Error in read loop: {e}")
                time.sleep(0.05)

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

            checksum_calculated = 0
            for b in packet[2:-1]:
                checksum_calculated ^= b
            if checksum_calculated != packet[-1]:
                log(f"Warning: Checksum mismatch for packet: {packet.hex(' ')}")
                continue

            self._parse_feedback_payload(packet)

    def control_head(self, yaw_deg, pitch_deg, duration_ms=50, logging=False):
        self.smoothed_yaw = ALPHA * yaw_deg + (1 - ALPHA) * self.smoothed_yaw
        self.smoothed_pitch = ALPHA * pitch_deg + (1 - ALPHA) * self.smoothed_pitch
        
        final_yaw_deg = max(YAW_MIN, min(YAW_MAX, self.smoothed_yaw))
        final_pitch_deg = max(PITCH_MIN, min(PITCH_MAX, self.smoothed_pitch))

        cmd = self.build_neck_position_command(final_yaw_deg, final_pitch_deg, duration_ms)
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(cmd)
            except Exception as e:
                log(f"Error writing head control command to serial port: {e}")
                return
            if logging:
                log(f"  Sent command: {cmd.hex(' ')}")
                log(f"  Target: yaw={final_yaw_deg:.2f}, pitch={final_pitch_deg:.2f}, duration={duration_ms}ms")
        else:
            log("Warning: Serial port not open to send head control command.")

    def _build_packet(self, payload):
        packet = bytearray([0xAA, 0x55, len(payload)]) + payload
        checksum = 0
        for b in packet[2:]:
            checksum ^= b
        packet.append(checksum)
        return packet

    def build_neck_position_command(self, yaw_deg=0.0, pitch_deg=0.0, duration_ms=50):
        yaw_val = int(yaw_deg * 10) # 0.1 deg units
        pitch_val = int(pitch_deg * 10)
        
        def to_bytes_le(val, length=2):
            return val.to_bytes(length, byteorder='little', signed=True)

        payload = bytearray([0x04, 0x08])
        payload += to_bytes_le(yaw_val)
        payload += to_bytes_le(pitch_val)
        payload += to_bytes_le(duration_ms, 2)
        payload += to_bytes_le(duration_ms, 2)
        return self._build_packet(payload)

    def _build_version_request(self, enable=True):
        payload = bytearray([REQUEST_EXTRA_INFO_CMD, 0x01, 0x1F if enable else 0x00])
        return self._build_packet(payload)

    def _parse_feedback_payload(self, packet):
        main_payload = packet[3:-1]

        i = 0
        found_version = False
        with self._feedback_data_lock:
            while i < len(main_payload):
                tag = main_payload[i]
                if i + 1 >= len(main_payload): break
                block_len = main_payload[i+1]
                if i + 2 + block_len > len(main_payload): break
                value = main_payload[i+2 : i+2+block_len]

                if tag == FEEDBACK_TAG_HARDWARE_VERSION and block_len == 4:
                    try:
                        name = value[2:].decode('ascii', errors='ignore')
                        self._feedback_data["Hardware Version"] = f"{name} v{value[1]}.{value[0]}"
                        found_version = True
                    except Exception:
                        self._feedback_data[f"TAG_0x{tag:02x}"] = value.hex(' ')
                elif tag == FEEDBACK_TAG_FIRMWARE_VERSION and block_len == 4:
                    try:
                        year, month, day = 2000 + value[1], value[2], value[3]
                        self._feedback_data["Firmware Build Date"] = f"{year}-{month:02d}-{day:02d}"
                        found_version = True
                    except Exception:
                        self._feedback_data[f"TAG_0x{tag:02x}"] = value.hex(' ')
                elif tag == FEEDBACK_TAG_HW_DROP_IR and block_len == 1:
                    self._feedback_data["HW Drop IR Type"] = value[0]
                elif tag == FEEDBACK_TAG_WHEEL_ENCODER and block_len == 8:
                    left = int.from_bytes(value[0:4], 'little', signed=True)
                    right = int.from_bytes(value[4:8], 'little', signed=True)
                    self._feedback_data["Wheel Encoder"] = {"left": left, "right": right}
                elif tag == FEEDBACK_TAG_NECK_ENCODER and block_len == 4:
                    yaw_raw = int.from_bytes(value[0:2], 'little', signed=True)
                    pitch_raw = int.from_bytes(value[2:4], 'little', signed=True)
                    self.current_neck_yaw_deg = yaw_raw / 10.0
                    self.current_neck_pitch_deg = pitch_raw / 10.0
                    self._feedback_data["Neck Encoder"] = {"yaw_deg": self.current_neck_yaw_deg, "pitch_deg": self.current_neck_pitch_deg}
                elif tag == FEEDBACK_TAG_ODOMETRY and block_len == 12:
                    x = int.from_bytes(value[0:4], 'little', signed=True)
                    y = int.from_bytes(value[4:8], 'little', signed=True)
                    theta = int.from_bytes(value[8:12], 'little', signed=True) / 10.0
                    self._feedback_data["Odometry"] = {"x_mm": x, "y_mm": y, "theta_deg": theta}
                elif tag == FEEDBACK_TAG_AVOID_TARGET and block_len == 8:
                    x = int.from_bytes(value[0:4], 'little', signed=True)
                    y = int.from_bytes(value[4:8], 'little', signed=True)
                    self._feedback_data["Avoid Target"] = {"x_mm": x, "y_mm": y}
                elif tag == FEEDBACK_TAG_ERROR_CODE and block_len == 3:
                    self._feedback_data["Error Code"] = value.hex(' ')
                else:
                    self._feedback_data[f"TAG_0x{tag:02x}"] = value.hex(' ')

                if found_version:
                    # store the raw packet only when we actually found version info
                    self._last_version_response_raw = packet

                i += 2 + block_len

            # Signal only when version information was found in this packet
            if found_version:
                self._version_info_ready.set()

    def get_all_feedback_data(self):
        with self._feedback_data_lock:
            return self._feedback_data.copy()

    def get_firmware_version(self, timeout=5):
        if not (self.serial_port and self.serial_port.is_open and self._running):
            return {"error": "Serial port not available or listener not running"}
        
        cmd = self._build_version_request(True)
        log(f"Sending version request: {cmd.hex(' ')}")
        self._version_info_ready.clear()
        try:
            self.serial_port.write(cmd)
        except Exception as e:
            msg = f"Error sending version request to serial port: {e}"
            log(msg)
            return {"error": msg}

        if not self._version_info_ready.wait(timeout=timeout):
            msg = f"Timeout waiting for version response (waited {timeout} s)."
            log(msg)
            return {"error": msg}
        
        return self.get_all_feedback_data()

    def destroy(self):
        self.stop_listening()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            log("Serial port closed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Control the robot's head and get firmware version.")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw angle in degrees for head control.")
    parser.add_argument("--pitch", type=float, default=0.0, help="Pitch angle in degrees for head control.")
    parser.add_argument("--duration", type=int, default=1000, help="Duration in milliseconds for head control.")
    parser.add_argument("--monitor-duration", type=int, default=10, 
                        help="Duration in seconds to monitor head position after command.")
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

        log(f"\n--- Monitoring Neck Positions for {args.monitor_duration} seconds (1 Hz) ---")
        for i in range(args.monitor_duration):
            if not head_controller._running:
                log("Connection to device lost. Stopping monitor.")
                break
            log(f"  Time {i+1}s: Current Neck Yaw={head_controller.current_neck_yaw_deg:.1f} deg, Pitch={head_controller.current_neck_pitch_deg:.1f} deg")
            time.sleep(1)
        log("-" * 60)

        head_controller.destroy()
    else:
        log("Exiting due to serial port not being available.")