import serial
import time

PORT = 'COM5'
BAUD_RATE = 921600

# 1. The Telemetry Request (The repeating packet you found)
HEARTBEAT_PACKET = bytearray([0x02, 0x01, 0x1E, 0xF3, 0xFF, 0x03])

# 2. The Motor Command (The wake-up packet you found)
COMMAND_PACKET = bytearray([
    0x02, 0x10, 0x60, 0x60, 0x60, 0x60, 0x7F, 0xFF, 
    0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF, 0x01, 0x01, 
    0xFF, 0xFF, 0xC1, 0x8D, 0x03
])

def stream_motor_telemetry():
    print(f"Connecting to clean port {PORT}...")
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=0)
        ser.dtr = True
        ser.rts = True
        time.sleep(0.1) # Give the chip 100ms to wake up
        
        print("[SUCCESS] Connected. Transmitting Heartbeat & Command (Press Ctrl+C to stop)...\n")
        
        while True:
            # 1. Ask the motor for its current status
            ser.write(HEARTBEAT_PACKET)
            
            # 2. Tell the motor to stay awake in MIT mode
            ser.write(COMMAND_PACKET)
            
            # Force the USB to send instantly
            ser.flush() 
            
            # 3. Read whatever the motor yells back
            time.sleep(0.02) # 50 Hz loop
            
            if ser.in_waiting > 0:
                raw_reply = ser.read(ser.in_waiting)
                readable_reply = " ".join([f"{b:02X}" for b in raw_reply])
                print(f"Live Telemetry -> {readable_reply}")
                
    except KeyboardInterrupt:
        print("\nStreaming paused by user.")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Port safely released.")

if __name__ == "__main__":
    stream_motor_telemetry()