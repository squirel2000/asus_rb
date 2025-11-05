import serial
import math
import argparse
import time

# Head control for the robot using serial communication, and the protocol defined in:
# https://asus.sharepoint.com/:w:/r/sites/ZenGimbal_Ver2.0/_layouts/15/Doc.aspx?sourcedoc=%7B563FB2AC-D8AA-4B42-B4BB-808C06ECD9F9%7D&file=TOBY%20communications%20protocol_beta(versionb0.91).docx&wdOrigin=TEAMS-WEB.p2p_ns.rwc&action=default&mobileredirect=true
#
# Usage: For example:
# python3 src/follow_user/scripts/utils/head_control.py --yaw 10.0 --pitch -5.0 --duration 1000

# Constants
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200
ALPHA = 0.1  # Smoothing factor for low-pass filter
# Yaw and pitch limits
YAW_MIN, YAW_MAX = -45.0, 45.0
PITCH_MIN, PITCH_MAX = -15.0, 55.0

class HeadController:
    def __init__(self, logger=None):
        self.logger = logger
        try:
            self.serial_port = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        except serial.SerialException as e:
            if self.logger:
                self.logger.error(f"Could not open serial port: {e}")
            else:
                print(f"Error: Could not open serial port: {e}")
            self.serial_port = None

        # Low-pass filter parameters
        self.smoothed_yaw = 0.0
        self.smoothed_pitch = 0.0

    def control_head(self, yaw, pitch, duration_ms=50):
        # Apply low-pass filter
        self.smoothed_yaw = ALPHA * yaw + (1 - ALPHA) * self.smoothed_yaw
        self.smoothed_pitch = ALPHA * pitch + (1 - ALPHA) * self.smoothed_pitch
        yaw_deg = math.degrees(self.smoothed_yaw)
        pitch_deg = math.degrees(self.smoothed_pitch)

        # Limit yaw and pitch ranges
        yaw_deg = max(YAW_MIN, min(YAW_MAX, yaw_deg))
        pitch_deg = max(PITCH_MIN, min(PITCH_MAX, pitch_deg))

        cmd = self.build_neck_position_command(yaw_deg, pitch_deg, duration_ms)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(cmd)
            if self.logger:
                self.logger.info(f"Sent head command: yaw={yaw_deg:.2f}, pitch={pitch_deg:.2f}")
            else:
                print(f"Sent command: yaw={yaw_deg:.2f}, pitch={pitch_deg:.2f}")

    def build_neck_position_command(self, yaw_deg=0.0, pitch_deg=0.0, duration_ms=50):
        # Convert degrees to 0.1° units
        yaw_val = int(yaw_deg * 10)
        pitch_val = int(pitch_deg * 10)

        # Convert to little-endian byte arrays (LSB first)
        def to_bytes_le(val, length=2):
            return val.to_bytes(length, byteorder='little', signed=True)

        # Payload (Sub-payload for Neck Position Control)
        payload = bytearray()
        payload += b'\x04'               # Header identifier
        payload += b'\x08'               # Length
        payload += to_bytes_le(yaw_val)  # Yaw position (0.1°)
        payload += to_bytes_le(pitch_val)# Pitch position (0.1°)
        payload += to_bytes_le(duration_ms, 2)  # Time Yaw (ms)
        payload += to_bytes_le(duration_ms, 2)  # Time Pitch (ms)

        # Packet = Header + Length + Payload + Checksum
        packet = bytearray([0xAA, 0x55])
        packet.append(len(payload))
        packet += payload

        # Compute XOR checksum (exclude headers)
        checksum = 0
        for b in packet[2:]:  # skip 0xAA, 0x55
            checksum ^= b
        packet.append(checksum)

        return packet

    def destroy(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Control the robot's head.")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw angle in degrees.")
    parser.add_argument("--pitch", type=float, default=0.0, help="Pitch angle in degrees.")
    parser.add_argument("--duration", type=int, default=1000, help="Duration in milliseconds.")
    args = parser.parse_args()

    head_controller = HeadController()
    if head_controller.serial_port:
        cmd = head_controller.build_neck_position_command(args.yaw, args.pitch, args.duration)
        head_controller.serial_port.write(cmd)
        print(f"Sent command: {cmd.hex(' ')}")
        response = head_controller.serial_port.read(64)
        print(f"Response data: {response.hex(' ')}" if response else "No response received")
        head_controller.destroy()
