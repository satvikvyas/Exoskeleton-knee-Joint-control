import serial
import struct
import time

FRAME_HEAD = 0x02
FRAME_TAIL = 0x03
COMM_GET_VALUES = 4

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

def main():
    try:
        ser = serial.Serial('COM6', 921600, timeout=0.1)
        time.sleep(0.2)
    except Exception as e:
        print("Failed:", e)
        return

    ser.reset_input_buffer()
    ser.write(build_frame(bytes([COMM_GET_VALUES])))
    
    head = ser.read(1)
    if not head or head[0] != FRAME_HEAD: 
        print("No head")
        return
    length_byte = ser.read(1)
    if not length_byte: 
        print("No len")
        return
    rest = ser.read(length_byte[0] + 3)
    
    payload = parse_frame(head + length_byte + rest)
    if payload:
        print(f"Payload length: {len(payload)}")
        # Print every 4-byte chunk
        for i in range(1, len(payload)-3, 4):
            val_int = struct.unpack('>i', payload[i:i+4])[0]
            val_uint = struct.unpack('>I', payload[i:i+4])[0]
            print(f"Offset {i}: Int={val_int}, UInt={val_uint}, Hex={payload[i:i+4].hex()}")
            
        print("\nCommon Offsets:")
        if len(payload) >= 58:
            print(f"45-48 (tachometer?): {struct.unpack('>i', payload[45:49])[0]}")
            print(f"49-52 (tach_abs?): {struct.unpack('>i', payload[49:53])[0]}")
            print(f"54-57 (pid_pos?): {struct.unpack('>i', payload[54:58])[0]}")
    else:
        print("Parse failed")
        
    ser.close()

if __name__ == '__main__':
    main()
