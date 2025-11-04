import serial, time

def build_neck_position_command(yaw_deg=5.0, pitch_deg=0.0, duration_ms=3000):
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


def send_command(serial_port="/dev/ttyACM0", baudrate=115200):
    cmd = build_neck_position_command(0.0, 0.0, 3000)

    with serial.Serial(serial_port, baudrate, timeout=1) as ser:
        start_time = time.time()
        ser.write(cmd)
        
        # Wait for callback or feedback packet
        response = ser.read(64) # Read feedback
        elapsed = time.time() - start_time
        
        print(f"Sent command: {cmd.hex(' ')}")
        print(f"Round-trip time: {elapsed*1000:.1f} ms")
        print(f"Response data: {response.hex(' ')}" if response else "No response received")


if __name__ == "__main__":
    send_command("/dev/ttyACM0", 115200)
