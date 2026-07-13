"""
CubeMars AK80-64 Servo-Mode UART control (via FTDI232 <-> motor serial port)

Protocol reference: CubeMars AK Series Module Driver User Manual, section 5.2.2
(Servo Mode Serial Message Protocol).

Frame format:
    [0x02][Data Length][Data Frame ... ][CRC High][CRC Low][0x03]

CRC is CRC-16/XMODEM (poly 0x1021, init 0x0000), computed over the
Data Frame bytes only (excludes head, length, tail, and the CRC itself).

IMPORTANT SAFETY NOTES:
- Confirm the motor is actually in SERVO mode (not MIT mode) via the
  CubeMars Upper Computer -> Mode Switch -> "Enter Servo Mode" before
  running this. Sending servo-protocol frames while the motor is in
  MIT mode (or vice versa) can damage the driver board per the manual's
  explicit warning.
- Confirm the baud rate in Upper Computer's "Serial Port Selection" field
  and update BAUD_RATE below if it differs from 115200.
- This script defaults to READ-ONLY telemetry (get_values). Motion
  commands (set_rpm / set_position) are provided but not called
  automatically -- call them deliberately once you've confirmed
  telemetry looks sane.
"""

import struct
import time
import serial

BAUD_RATE = 921600   # confirmed via CubeMars master guide / community implementations
SERIAL_PORT = 'COM6'  # adjust to match your FTDI device

FRAME_HEAD = 0x02
FRAME_TAIL = 0x03

# Command IDs (COMM_PACKET_ID enum from the manual)
COMM_GET_VALUES = 4
COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8
COMM_SET_POS = 9


def crc16_xmodem(data: bytes) -> int:
    """CRC-16/XMODEM: poly=0x1021, init=0x0000, no reflect, no xorout."""
    crc = 0
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(payload: bytes) -> bytes:
    length = len(payload)
    if length > 256:
        raise ValueError("Payload too long for single-byte length field")
    crc = crc16_xmodem(payload)
    frame = bytearray()
    frame.append(FRAME_HEAD)
    frame.append(length)
    frame.extend(payload)
    frame.append((crc >> 8) & 0xFF)
    frame.append(crc & 0xFF)
    frame.append(FRAME_TAIL)
    return bytes(frame)


def parse_frame(raw: bytes):
    """Validate and extract the payload from a received frame."""
    if len(raw) < 5:
        raise ValueError(f"Frame too short: {raw.hex()}")
    if raw[0] != FRAME_HEAD:
        raise ValueError(f"Bad frame head: {raw.hex()}")
    length = raw[1]
    payload = raw[2:2 + length]
    crc_recv = (raw[2 + length] << 8) | raw[3 + length]
    tail = raw[4 + length]
    if tail != FRAME_TAIL:
        raise ValueError(f"Bad frame tail: {raw.hex()}")
    crc_calc = crc16_xmodem(payload)
    if crc_calc != crc_recv:
        raise ValueError(f"CRC mismatch: got {crc_recv:04X}, expected {crc_calc:04X}")
    return payload


def send_command(ser: serial.Serial, payload: bytes, expect_reply: bool = True, timeout=0.5):
    frame = build_frame(payload)
    ser.reset_input_buffer()
    ser.write(frame)
    if not expect_reply:
        return None

    ser.timeout = timeout
    # Read the head byte, then the length byte, then payload+crc+tail
    head = ser.read(1)
    if len(head) == 0:
        return None  # timeout, no reply
    if head[0] != FRAME_HEAD:
        raise ValueError(f"Unexpected byte where frame head expected: {head.hex()}")
    length_byte = ser.read(1)
    if len(length_byte) == 0:
        return None
    length = length_byte[0]
    rest = ser.read(length + 3)  # payload + 2 crc bytes + tail
    if len(rest) < length + 3:
        return None  # incomplete frame
    raw = head + length_byte + rest
    return parse_frame(raw)


def get_values(ser: serial.Serial):
    """COMM_GET_VALUES: request full telemetry frame. Read-only, no motion."""
    payload = bytes([COMM_GET_VALUES])
    reply = send_command(ser, payload)
    if reply is None:
        print("No reply to GET_VALUES (timeout).")
        return None

    ind = 1  # reply[0] is the echoed command id
    mos_temp = struct.unpack('>h', reply[ind:ind+2])[0] / 10.0; ind += 2
    motor_temp = struct.unpack('>h', reply[ind:ind+2])[0] / 10.0; ind += 2
    output_current = struct.unpack('>i', reply[ind:ind+4])[0] / 100.0; ind += 4
    input_current = struct.unpack('>i', reply[ind:ind+4])[0] / 100.0; ind += 4
    id_current = struct.unpack('>i', reply[ind:ind+4])[0] / 100.0; ind += 4
    iq_current = struct.unpack('>i', reply[ind:ind+4])[0] / 100.0; ind += 4
    duty_now = struct.unpack('>h', reply[ind:ind+2])[0] / 1000.0; ind += 2
    erpm = struct.unpack('>i', reply[ind:ind+4])[0]; ind += 4          # electrical RPM (velocity)
    v_in = struct.unpack('>h', reply[ind:ind+2])[0] / 10.0; ind += 2

    print(f"MOS temp: {mos_temp:.1f} C | Motor temp: {motor_temp:.1f} C | "
          f"Output current: {output_current:.2f} A | Input current: {input_current:.2f} A | "
          f"Duty: {duty_now:.3f} | Velocity: {erpm} ERPM | V_in: {v_in:.1f} V")

    # Position (pid_pos_now) sits further into the payload, after amp/watt-hour
    # and tachometer fields. Only parse it if the reply is long enough --
    # length varies slightly across firmware builds.
    pos_offset = ind + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 1  # skip amp_hours(x2), watt_hours(x2), tachometer(x2), fault_code
    if len(reply) >= pos_offset + 4:
        position = struct.unpack('>i', reply[pos_offset:pos_offset+4])[0] / 1_000_000.0
        print(f"Position: {position:.2f} deg")
    else:
        print(f"(Reply too short for position field: got {len(reply)} bytes, need {pos_offset + 4})")

    return reply


def set_rpm(ser: serial.Serial, erpm: int):
    """COMM_SET_RPM: electrical RPM, direct int32, no scaling."""
    payload = bytes([COMM_SET_RPM]) + struct.pack('>i', int(erpm))
    send_command(ser, payload, expect_reply=False)


def set_position(ser: serial.Serial, degrees: float):
    """COMM_SET_POS: position in degrees, scaled by 1,000,000, int32."""
    payload = bytes([COMM_SET_POS]) + struct.pack('>i', int(degrees * 1_000_000))
    send_command(ser, payload, expect_reply=False)


if __name__ == "__main__":
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
    time.sleep(0.2)  # let the port settle

    try:
        print("Querying motor telemetry (read-only, no motion)...")
        for _ in range(5):
            get_values(ser)
            time.sleep(0.2)

        # --- Motion commands: opt-in only, uncomment deliberately ---
        # print("Sending small RPM test...")
        # set_rpm(ser, 500)   # 500 electrical RPM
        # time.sleep(2)
        # set_rpm(ser, 0)     # stop

    finally:
        ser.close()