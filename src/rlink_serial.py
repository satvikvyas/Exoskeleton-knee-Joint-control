import serial
import time
import binascii

# ==========================================
# 1. HARDWARE CONFIGURATION
# ==========================================
PORT = 'COM5'
BAUD_RATE = 921600  # The standard RLink V2 operating speed

# ==========================================
# 2. THE SECRET BYTE PAYLOAD
# ==========================================
# This is where the magic happens. 
# You will replace this string with the EXACT hex bytes you sniff 
# from the Free Serial Analyzer when you click "Enable" in the CubeMars app.
# 
# Example format: "AA 55 01 FF FF FF FF FF FF FF FC 8B"


WAKE_COMMAND_HEX = "FF FF FF FF FF FF FF F" # Placeholder - Needs RLink Wrapper Bytes!



def connect_and_send():
    print(f"Attempting to open {PORT} at {BAUD_RATE} baud...")
    
    try:
        # Open the raw serial port
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1  # 100ms timeout for non-blocking reads
        )
        print("[SUCCESS] Port opened successfully.\n")
        
    except Exception as e:
        print(f"[FAILED] Could not open port. Is the CubeMars app still open?\nError: {e}")
        return

    try:
        # 1. Format the string into raw binary bytes
        # Removes spaces and converts hex text to actual computer bytes
        clean_hex = WAKE_COMMAND_HEX.replace(" ", "")
        binary_payload = binascii.unhexlify(clean_hex)
        
        # 2. Transmit the data to the RLink V2
        print(f"Transmitting bytes: {WAKE_COMMAND_HEX}")
        ser.write(binary_payload)
        
        # Force the USB buffer to push the data immediately
        ser.flush() 
        
        # 3. Wait a tiny fraction of a second for the motor to reply
        time.sleep(0.05)
        
        # 4. Read the reply from the RLink V2
        if ser.in_waiting > 0:
            raw_reply = ser.read(ser.in_waiting)
            
            # Convert the raw computer bytes back into readable hex text
            readable_reply = " ".join([f"{b:02X}" for b in raw_reply])
            
            print("\n[SUCCESS] Motor Replied!")
            print(f"Raw Reply Bytes: {readable_reply}")
        else:
            print("\n[SILENCE] No reply received. Check your wrapper bytes.")
            
    except Exception as e:
        print(f"Error during transmission: {e}")
        
    finally:
        # Always close the port safely so it doesn't get permanently locked
        ser.close()
        print("\nPort closed safely.")

# Execute the function
if __name__ == "__main__":
    connect_and_send()