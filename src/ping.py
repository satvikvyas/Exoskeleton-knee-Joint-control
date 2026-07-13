import can
import time

def ping_motor():
    try:
        # 1. Open the CAN Bus
        bus = can.interface.Bus(interface='slcan', channel='COM5', bitrate=1000000, tty_baudrate=921600)
        print("Port COM5 open.")
        
        # 2. The official CubeMars "Enter Motor Mode" byte sequence
        # This tells the motor to wake up and reply with its current state
        wake_data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
        wake_msg = can.Message(arbitration_id=0x01, data=wake_data, is_extended_id=False)
        
        # 3. Transmit the wake-up frame
        print("Transmitting Wake-Up Frame...")
        bus.send(wake_msg)
        
        # 4. Listen for the immediate telemetry reply
        reply = bus.recv(timeout=2.0)
        
        if reply is not None:
            print(f"\n[SUCCESS] Motor Replied!")
            print(f"CAN ID: {hex(reply.arbitration_id)}")
            print(f"Raw Data: {reply.data.hex()}")
        else:
            print("\n[FAILED] Bus is silent. The motor did not reply.")
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'bus' in locals():
            bus.shutdown()
            print("Port closed.")

ping_motor()