import struct
import time
import serial

# ==========================================
# 1. HARDWARE CONSTANTS
# ==========================================
SERIAL_PORT = 'COM6'
BAUD_RATE = 921600
TARGET_DT = 0.01  # 100 Hz

INCREASING_POS_IS_CW = False  # <-- flip if wrong. Informational only -- this
                               # joint has ~160 deg of travel, not a full
                               # rotation, so direction can't change the path,
                               # only tells you which way it'll actually turn.

KT_NM_PER_A = 0.136  # torque constant, for live diagnostic display

# ==========================================
# 2. SERVO-MODE UART PROTOCOL
# ==========================================
FRAME_HEAD = 0x02
FRAME_TAIL = 0x03
COMM_GET_VALUES = 4
COMM_SET_CURRENT = 6
COMM_SET_POS = 9

def crc16_xmodem(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000: crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else: crc = (crc << 1) & 0xFFFF
    return crc

def build_frame(payload: bytes) -> bytes:
    crc = crc16_xmodem(payload)
    frame = bytearray([FRAME_HEAD, len(payload)])
    frame.extend(payload)
    frame.extend([(crc >> 8) & 0xFF, crc & 0xFF, FRAME_TAIL])
    return bytes(frame)

def parse_frame(raw: bytes):
    if len(raw) < 5 or raw[0] != FRAME_HEAD: return None
    length = raw[1]
    payload = raw[2:2 + length]
    crc_recv = (raw[2 + length] << 8) | raw[3 + length]
    if crc16_xmodem(payload) != crc_recv: return None
    return payload

def send_command(ser: serial.Serial, payload: bytes, expect_reply: bool = True):
    ser.reset_input_buffer()
    ser.write(build_frame(payload))
    if not expect_reply: return None
    ser.timeout = 0.01
    head = ser.read(1)
    if not head or head[0] != FRAME_HEAD: return None
    length_byte = ser.read(1)
    if not length_byte: return None
    rest = ser.read(length_byte[0] + 3)
    return parse_frame(head + length_byte + rest)

def set_current(ser: serial.Serial, amps: float):
    payload = bytes([COMM_SET_CURRENT]) + struct.pack('>i', int(amps * 1000))
    send_command(ser, payload, expect_reply=False)

def set_position(ser: serial.Serial, degrees: float):
    payload = bytes([COMM_SET_POS]) + struct.pack('>i', int(degrees * 1_000_000))
    send_command(ser, payload, expect_reply=False)

def get_state(ser: serial.Serial):
    """Returns (position_deg, motor_current_a) or None on miss."""
    reply = send_command(ser, bytes([COMM_GET_VALUES]))
    if reply is None or len(reply) < 58:
        return None
    motor_current = struct.unpack('>i', reply[5:9])[0] / 100.0
    pos_deg = struct.unpack('>i', reply[54:58])[0] / 1_000_000.0
    return pos_deg, motor_current

# ==========================================
# 3. MAIN SCRIPT
# ==========================================
def main():
    print("Opening serial port...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        time.sleep(0.2)
    except Exception as e:
        print(f"Failed to open port: {e}")
        return

    print("\n--- POSITION CONTROL MODE ---")
    state = get_state(ser)
    if state is None:
        print("No telemetry reply -- confirm motor is powered and in Servo mode.")
        ser.close()
        return
    current_pos, _ = state
    print(f"Current position: {current_pos:.2f} deg")

    try:
        target_deg = float(input("Enter target position (deg): "))
    except ValueError:
        print("Invalid number. Exiting.")
        return

    direction_input = input("Direction (cw for Clockwise, ccw/acw for Anti-Clockwise) [cw]: ").strip().lower()
    want_cw = direction_input not in ['ccw', 'acw']

    # Force the actual path: pid_pos_now is a multi-turn absolute value with
    # no built-in wraparound, so the firmware just PIDs straight to whatever
    # number you send -- it won't pick a direction for you. Add/subtract a
    # full turn so the resulting delta's sign matches what you asked for.
    target_mod = target_deg % 360.0
    current_mod = current_pos % 360.0
    forward_delta = (target_mod - current_mod) % 360.0    # CW-increasing convention, in [0, 360)
    backward_delta = forward_delta - 360.0                 # in (-360, 0]

    if forward_delta == 0.0:
        delta = 0.0
    elif want_cw == INCREASING_POS_IS_CW:
        delta = forward_delta
    else:
        delta = backward_delta

    target_deg = current_pos + delta

    print(f"\nMoving to {target_deg:.2f} deg (absolute) via {'CW' if want_cw else 'CCW'}. Press Ctrl+C to stop.")
    time.sleep(0.5)

    try:
        while True:
            loop_start = time.perf_counter()

            set_position(ser, target_deg)

            state = get_state(ser)
            if state is not None:
                actual_pos, motor_current = state
                torque = motor_current * KT_NM_PER_A
                print(f"\rPos: {actual_pos:7.2f} deg | Current: {motor_current:6.2f} A | "
                      f"Torque: {torque:6.2f} Nm", end="")

            elapsed = time.perf_counter() - loop_start
            sleep_time = TARGET_DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nCtrl+C detected.")

    finally:
        print("Zeroing current and closing port...")
        set_current(ser, 0.0)
        time.sleep(0.05)
        ser.close()
        print("Motor safely deactivated.")

if __name__ == "__main__":
    main()