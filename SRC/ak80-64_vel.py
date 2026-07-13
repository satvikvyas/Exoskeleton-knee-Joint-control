import struct
import time
import math
import serial
import multiprocessing as mp
import threading
from collections import deque
# pyrefly: ignore [missing-import]
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ==========================================
# 1. HARDWARE CONSTANTS
# ==========================================
SERIAL_PORT = 'COM6'
BAUD_RATE = 921600
POLE_PAIRS = 21
GEAR_RATIO = 64.0

TARGET_DT = 0.01  # 100 Hz

# ==========================================
# 2. SERVO-MODE UART PROTOCOL
# ==========================================
FRAME_HEAD = 0x02
FRAME_TAIL = 0x03
COMM_GET_VALUES = 4
COMM_SET_CURRENT = 6
COMM_SET_RPM = 8      

# VESC Fault Codes Translation Dictionary
FAULT_CODES = {
    0: "NONE", 1: "OVER_VOLTAGE", 2: "UNDER_VOLTAGE", 3: "DRV_FAULT",
    4: "ABS_OVER_CURRENT", 5: "OVER_TEMP_FET", 6: "OVER_TEMP_MOTOR"
}

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

def set_erpm(ser: serial.Serial, erpm: float):
    payload = bytes([COMM_SET_RPM]) + struct.pack('>i', int(erpm))
    send_command(ser, payload, expect_reply=False)

def get_state(ser: serial.Serial):
    reply = send_command(ser, bytes([COMM_GET_VALUES]))
    if reply is None or len(reply) < 55: return None

    # Current (Offset 5)
    motor_current = struct.unpack('>i', reply[5:9])[0] / 100.0
    
    # ERPM (Offset 23)
    erpm = struct.unpack('>i', reply[23:27])[0]
    
    # Tachometer for absolute continuous position (Offset 45)
    tacho_ticks = struct.unpack('>i', reply[45:49])[0]
    
    # Fault Code (Offset 53)
    fault_code_int = reply[53]
    
    # Kinematic Conversions
    motor_rpm = erpm / POLE_PAIRS
    limb_rad_s = motor_rpm * (2 * math.pi / 60.0) / GEAR_RATIO
    
    rotor_deg = (tacho_ticks / 6.0) / POLE_PAIRS * 360.0
    limb_deg = rotor_deg / GEAR_RATIO

    return limb_deg, motor_rpm, limb_rad_s, motor_current, fault_code_int

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.val = 0.0
    def update(self, new_val):
        self.val = (self.alpha * new_val) + ((1.0 - self.alpha) * self.val)
        return self.val

# ==========================================
# 3. MULTIPROCESSING UI DASHBOARD
# ==========================================
def telemetry_dashboard(data_queue):
    """Runs in an isolated CPU core to prevent graphing from lagging the motor."""
    history_len = 200
    t_data = deque([0]*history_len, maxlen=history_len)
    pos_data = deque([0]*history_len, maxlen=history_len)
    rpm_data = deque([0]*history_len, maxlen=history_len)
    curr_data = deque([0]*history_len, maxlen=history_len)

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    fig.canvas.manager.set_window_title('AK80-64 Live Telemetry')
    
    line_pos, = ax1.plot(t_data, pos_data, 'b-', label='Limb Angle (deg)')
    line_rpm, = ax2.plot(t_data, rpm_data, 'g-', label='Rotor RPM')
    line_curr, = ax3.plot(t_data, curr_data, 'r-', label='Phase Current (LPF) (A)')

    ax1.set_ylabel('Position (deg)')
    ax1.legend(loc='upper left')
    ax1.grid(True)

    ax2.set_ylabel('Rotor RPM')
    ax2.legend(loc='upper left')
    ax2.grid(True)

    ax3.set_ylabel('Current (A)')
    ax3.set_xlabel('Time (seconds)')
    ax3.legend(loc='upper left')
    ax3.grid(True)

    plt.tight_layout()

    def update(frame):
        while not data_queue.empty():
            try:
                t, pos, rpm, curr = data_queue.get_nowait()
                t_data.append(t)
                pos_data.append(pos)
                rpm_data.append(rpm)
                curr_data.append(curr)
            except Exception:
                break

        current_time = t_data[-1]
        ax1.set_xlim(current_time - 2.0, current_time) 
        
        # Auto-scale Y axes slightly based on recent data
        if max(pos_data) != min(pos_data):
            ax1.set_ylim(min(pos_data)-10, max(pos_data)+10)
        ax2.set_ylim(min(rpm_data)-100, max(rpm_data)+100)
        ax3.set_ylim(-15, 15)

        line_pos.set_data(t_data, pos_data)
        line_rpm.set_data(t_data, rpm_data)
        line_curr.set_data(t_data, curr_data)
        
        return line_pos, line_rpm, line_curr

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, save_count=10)
    plt.show()

# ==========================================
# 4. OOP CONTROL LOOP
# ==========================================
class VelocityControlLoop(threading.Thread):
    def __init__(self, target_limb_vel_rad_s, want_cw, ser=None, send_cmd_func=None):
        super().__init__()
        self.target_limb_vel_rad_s = target_limb_vel_rad_s
        self.want_cw = want_cw
        self.ser = ser
        self.send_cmd_func = send_cmd_func
        self.running = False
        self.daemon = True

    def stop(self):
        self.running = False

    def send_vel_command(self, erpm):
        if self.send_cmd_func:
            self.send_cmd_func({'type': 'vel_raw', 'val': erpm})
        elif self.ser:
            set_erpm(self.ser, erpm)

    def run(self):
        self.running = True

        target_vel = self.target_limb_vel_rad_s
        if self.want_cw:
            target_vel = -abs(target_vel)
        else:
            target_vel = abs(target_vel)

        target_rotor_vel = target_vel * GEAR_RATIO
        target_rotor_rpm = target_rotor_vel * (60.0 / (2.0 * math.pi))
        target_erpm = target_rotor_rpm * POLE_PAIRS

        print(f"\nCommanding Limb: {target_vel:.2f} rad/s")
        print("Loop Running. Press Ctrl+C or trigger stop().")
        time.sleep(1.0)

        current_filter = LowPassFilter(alpha=0.3)
        start_time = time.perf_counter()

        try:
            while self.running:
                loop_start = time.perf_counter()
                t_elapsed = loop_start - start_time

                # 1. Command Speed
                self.send_vel_command(target_erpm)

                # 2. Read Telemetry (Standalone mode only)
                if self.ser:
                    state = get_state(self.ser)
                    if state is not None:
                        limb_deg, rotor_rpm, actual_vel_rad_s, current, fault_code = state
                        filtered_current = current_filter.update(current)
                        
                        if fault_code != 0:
                            fault_name = FAULT_CODES.get(fault_code, f"UNKNOWN ({fault_code})")
                            print(f"\n[!] HARDWARE FAULT DETECTED: {fault_name} [!]")
                        else:
                            print(f"\rVel: {actual_vel_rad_s:6.2f} rad/s | Cur (LPF): {filtered_current:5.2f} A | Fault: NONE  ", end="")

                # 3. Precision Pacing
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
# 5. STANDALONE SCRIPT
# ==========================================
def main():
    # Setup the isolated data pipeline for the graph
    data_queue = mp.Queue(maxsize=100) 
    ui_process = mp.Process(target=telemetry_dashboard, args=(data_queue,))
    ui_process.daemon = True 
    ui_process.start()

    print("Opening serial port...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        time.sleep(0.2)
    except Exception as e:
        print(f"Failed to open port: {e}")
        return

    print("\n--- VELOCITY CONTROL WITH TELEMETRY ---")
    try:
        target_limb_vel = float(input("Enter target limb velocity (rad/s): "))
    except ValueError:
        print("Invalid number. Exiting.")
        return

    direction_input = input("Enter direction (cw for Clockwise, ccw/acw for Anti-Clockwise) [cw]: ").strip().lower()
    want_cw = direction_input not in ['ccw', 'acw']

    loop = VelocityControlLoop(target_limb_vel, want_cw, ser=ser)
    loop.start()

    try:
        while loop.is_alive():
            time.sleep(0.1)
            # Check if user closed the graph window
            if not ui_process.is_alive():
                print("\nGraph window closed. Shutting down.")
                loop.stop()
                break
    except KeyboardInterrupt:
        loop.stop()
        loop.join()

    print("\nZeroing current and closing port...")
    ser.close()
    if ui_process.is_alive():
        ui_process.terminate()
        ui_process.join()
    print("Motor safely deactivated.")

if __name__ == "__main__":
    mp.freeze_support() 
    main()