import struct
import time
import serial

# ==========================================
# 1. HARDWARE CONSTANTS
# ==========================================
SERIAL_PORT = 'COM6'
BAUD_RATE = 921600

# ==========================================
# 2. PROTOCOL
# ==========================================
FRAME_HEAD = 0x02
FRAME_TAIL = 0x03
COMM_SET_POS_ORIGIN = 95

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

def send_command(ser: serial.Serial, payload: bytes):
    ser.reset_input_buffer()
    ser.write(build_frame(payload))
    time.sleep(0.1)

def set_origin(ser: serial.Serial, mode: int):
    """
    Mode 0: Temporary origin (lost on reboot)
    Mode 1: Permanent origin (saved to memory)
    Mode 2: Restore default zero point
    """
    payload = bytes([COMM_SET_POS_ORIGIN, mode])
    send_command(ser, payload)

# ==========================================
# 3. MAIN SCRIPT
# ==========================================
def main():
    print("Opening serial port...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(0.2)
    except Exception as e:
        print(f"Failed to open port: {e}")
        return

    print("\n--- SET MOTOR ORIGIN ---")
    print("0: Set temporary origin (lost on reboot)")
    print("1: Set permanent origin (saved to memory)")
    print("2: Restore default zero point")
    
    try:
        mode_input = input("\nEnter mode [0/1/2] (Default 0): ").strip()
        if not mode_input:
            mode = 0
        else:
            mode = int(mode_input)
            
        if mode not in [0, 1, 2]:
            print("Invalid mode. Exiting.")
            return
            
    except ValueError:
        print("Invalid number. Exiting.")
        return

    print(f"\nSetting motor origin (Mode {mode})...")
    set_origin(ser, mode)
    print("Done! If you set a permanent origin, you may need to power cycle the motor.")
    
    ser.close()

if __name__ == "__main__":
    main()
