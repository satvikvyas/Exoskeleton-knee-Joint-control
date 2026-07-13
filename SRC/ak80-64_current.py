import struct
import time
import serial
import threading

# ==========================================
# 1. HARDWARE CONSTANTS
# ==========================================
SERIAL_PORT = 'COM6'
BAUD_RATE = 921600

TARGET_DT = 0.01  # 100 Hz

INCREASING_POS_IS_CW = False  # <-- CONFIRM, flip if wrong

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
    if len(raw) < length + 5: return None  # Handle truncated packets
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
    reply = send_command(ser, bytes([COMM_GET_VALUES]))
    if reply is None or len(reply) < 58: return None
    ind = 54
    pos_deg = struct.unpack('>i', reply[ind:ind+4])[0] / 1_000_000.0
    return pos_deg

# ==========================================
# 3. OOP CONTROL LOOP
# ==========================================
class CurrentControlLoop(threading.Thread):
    def __init__(self, target_amps, want_cw, ser=None, send_cmd_func=None):
        super().__init__()
        self.target_amps = target_amps
        self.want_cw = want_cw
        self.ser = ser
        self.send_cmd_func = send_cmd_func
        self.running = False
        self.daemon = True

    def stop(self):
        self.running = False

    def send_current_command(self, amps):
        if self.send_cmd_func:
            self.send_cmd_func({'type': 'cur', 'val': amps})
        elif self.ser:
            set_current(self.ser, amps)

    def run(self):
        self.running = True
        
        target = self.target_amps
        if self.want_cw:
            target = -abs(target)
        else:
            target = abs(target)

        print(f"\nCommanding Current: {target:.2f} A")
        print("Loop Running. Press Ctrl+C or trigger stop().")
        time.sleep(0.5)

        try:
            while self.running:
                loop_start = time.perf_counter()

                self.send_current_command(target)

                if self.ser:
                    state = get_state(self.ser)
                    if state is not None:
                        actual_pos, actual_cur = state
                        print(f"\rActual position: {actual_pos:.2f} deg | Current: {actual_cur:.2f} A", end="")

                elapsed = time.perf_counter() - loop_start
                sleep_time = TARGET_DT - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n\nCtrl+C detected (Thread Interrupted).")
        finally:
            print("\nZeroing current and ending loop...")
            if self.send_cmd_func:
                self.send_cmd_func({'type': 'stop'})
            elif self.ser:
                set_current(self.ser, 0.0)
                time.sleep(0.05)


# ==========================================
# 4. STANDALONE SCRIPT
# ==========================================
def main():
    print("Opening serial port...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        time.sleep(0.2)
    except Exception as e:
        print(f"Failed to open port: {e}")
        return

    print("\n--- TORQUE (CURRENT) CONTROL MODE ---")
    try:
        target_amps = float(input("Enter target current (Amps): "))
    except ValueError:
        print("Invalid number. Exiting.")
        return

    direction_input = input("Enter direction (cw for Clockwise, ccw/acw for Anti-Clockwise) [cw]: ").strip().lower()
    want_cw = direction_input not in ['ccw', 'acw']

    loop = CurrentControlLoop(target_amps, want_cw, ser=ser)
    loop.start()

    try:
        while loop.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        loop.stop()
        loop.join()

    ser.close()
    print("Motor safely deactivated.")

if __name__ == "__main__":
    main()