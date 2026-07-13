import struct
import time
import serial
import numpy as np

# ==========================================
# 1. HARDWARE & SAFETY CONSTANTS
# ==========================================
# Target is pacing, but actual DT is dynamically calculated now
TARGET_DT = 0.01       
MAX_TORQUE = 8.0        

# HARDWARE JOINT LIMITS (Safety stops for a knee)
# Adjust these based on how your encoder zeros out.
MIN_SAFE_ANGLE = -0.1       # radians (approx -5 degrees)
MAX_SAFE_ANGLE = 2.0        # radians (approx 115 degrees)

SERIAL_PORT = 'COM6'   
BAUD_RATE = 921600           

TORQUE_CONSTANT_NM_PER_A = 0.136
POLE_PAIRS = 21 
GEAR_RATIO = 64.0  

# ==========================================
# 2. SERVO-MODE UART PROTOCOL
# ==========================================
FRAME_HEAD = 0x02
FRAME_TAIL = 0x03

COMM_GET_VALUES = 4
COMM_SET_CURRENT = 6

def crc16_xmodem(data: bytes) -> int:
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
    crc = crc16_xmodem(payload)
    frame = bytearray([FRAME_HEAD, length])
    frame.extend(payload)
    frame.append((crc >> 8) & 0xFF)
    frame.append(crc & 0xFF)
    frame.append(FRAME_TAIL)
    return bytes(frame)

def parse_frame(raw: bytes):
    if len(raw) < 5 or raw[0] != FRAME_HEAD:
        raise ValueError(f"Bad frame: {raw.hex()}")
    length = raw[1]
    payload = raw[2:2 + length]
    crc_recv = (raw[2 + length] << 8) | raw[3 + length]
    if raw[4 + length] != FRAME_TAIL:
        raise ValueError(f"Bad frame tail: {raw.hex()}")
    if crc16_xmodem(payload) != crc_recv:
        raise ValueError("CRC mismatch")
    return payload

def send_command(ser: serial.Serial, payload: bytes, expect_reply: bool = True, timeout: float = 0.01):
    ser.reset_input_buffer()
    ser.write(build_frame(payload))
    if not expect_reply:
        return None
    ser.timeout = timeout
    head = ser.read(1)
    if not head or head[0] != FRAME_HEAD:
        return None
    length_byte = ser.read(1)
    if not length_byte:
        return None
    length = length_byte[0]
    rest = ser.read(length + 3)
    if len(rest) < length + 3:
        return None
    try:
        return parse_frame(head + length_byte + rest)
    except ValueError:
        return None

def set_current(ser: serial.Serial, amps: float):
    payload = bytes([COMM_SET_CURRENT]) + struct.pack('>i', int(amps * 1000))
    send_command(ser, payload, expect_reply=False)

def get_state(ser: serial.Serial):
    reply = send_command(ser, bytes([COMM_GET_VALUES]))
    if reply is None:
        return None

    ind = 1
    ind += 4  
    motor_current = struct.unpack('>i', reply[ind:ind+4])[0] / 100.0; ind += 4
    ind += 14  
    erpm = struct.unpack('>i', reply[ind:ind+4])[0]; ind += 4
    ind += 29  
    
    if len(reply) < ind + 4:
        return None
        
    pos_deg = struct.unpack('>i', reply[ind:ind+4])[0] / 1_000_000.0
    position_rad = np.radians(pos_deg)

    motor_rpm = erpm / POLE_PAIRS
    motor_rad_s = motor_rpm * (2 * np.pi / 60.0)
    velocity_rad_s = motor_rad_s / GEAR_RATIO

    return position_rad, velocity_rad_s, motor_current

# ==========================================
# 3. CONTROL ARCHITECTURE CLASSES
# ==========================================
class AdmittanceReferenceModel:
    def __init__(self, Mv, Bv, Kv):
        self.Mv = Mv
        self.Bv = Bv
        self.Kv = Kv
        self.theta_m = 0.0
        self.omega_m = 0.0

    def step(self, tau_human, r_baseline, dt):
        alpha_m = (tau_human - (self.Bv * self.omega_m) - (self.Kv * (self.theta_m - r_baseline))) / self.Mv
        self.omega_m += alpha_m * dt
        self.theta_m += self.omega_m * dt
        
        # Absolute safety clamp on the virtual model
        self.theta_m = np.clip(self.theta_m, MIN_SAFE_ANGLE, MAX_SAFE_ANGLE)
        return self.theta_m, self.omega_m

class MRAC_Controller:
    def __init__(self, ag, sigma, lam):
        self.ag = ag
        self.sigma = sigma
        self.lam = lam
        self.Kp = 15.0
        self.Kd = 1.0

    def calculate_torque(self, theta_actual, omega_actual, theta_model, omega_model, saturated, dt):
        e_pos = theta_actual - theta_model
        e_vel = omega_actual - omega_model
        s = e_pos + (self.lam * e_vel)
        abs_s = abs(s)

        if not saturated:
            self.Kp += (self.ag * s * theta_actual - self.sigma * abs_s * self.Kp) * dt
            self.Kd += (self.ag * s * omega_actual - self.sigma * abs_s * self.Kd) * dt

        self.Kp = np.clip(self.Kp, 0.0, 40.0)
        self.Kd = np.clip(self.Kd, 0.1, 5.0)

        tau_cmd = -(self.Kp * e_pos) - (self.Kd * e_vel)
        return tau_cmd

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.val = 0.0

    def update(self, new_val):
        self.val = (self.alpha * new_val) + ((1.0 - self.alpha) * self.val)
        return self.val

# ==========================================
# 4. MAIN REAL-TIME LOOP
# ==========================================
def main():
    print("Opening serial port...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        time.sleep(0.2)
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return

    # Note: DT removed from init. Passed dynamically during loop.
    adm_rm = AdmittanceReferenceModel(Mv=0.25, Bv=2.5, Kv=8.0)
    mrac = MRAC_Controller(ag=2.0, sigma=0.1, lam=0.5)
    vel_filter = LowPassFilter(alpha=0.2)

    r_baseline = 0.0  # Align baseline with 0 to prevent immediate snap on boot
    prev_vel = 0.0
    tau_cmd = 0.0

    loop_count = 0
    rate_check_start = time.perf_counter()
    prev_time = time.perf_counter()

    try:
        print("Starting control loop with dynamic time stepping...")

        while True:
            # -- DYNAMIC TIME STEPPING --
            current_time = time.perf_counter()
            dt = current_time - prev_time
            if dt <= 0.0:
                dt = 0.001 # Prevent divide-by-zero on ultra-fast OS ticks
            prev_time = current_time

            # 1. READ SENSOR DATA
            state = get_state(ser)
            if state is not None:
                actual_pos, raw_vel, motor_current = state
                actual_vel = vel_filter.update(raw_vel)
                actual_torque = motor_current * TORQUE_CONSTANT_NM_PER_A
            else:
                actual_vel = prev_vel
                actual_torque = 0.0
                actual_pos = locals().get('actual_pos', 0.0)
            prev_vel = actual_vel

            # 2. SENSORLESS ESTIMATION WITH DEADBAND
            tau_friction = 0.05 * actual_vel
            tau_human_est = -actual_torque - tau_friction
            
            # Prevent sensor noise from causing the admittance model to wander
            if abs(tau_human_est) < 0.25: 
                tau_human_est = 0.0

            # 3. OUTER LOOP: ADMITTANCE (Passed dynamic dt)
            theta_m, omega_m = adm_rm.step(tau_human_est, r_baseline, dt)

            # 4. INNER LOOP: MRAC ADAPTATION (Passed dynamic dt)
            is_saturated = (tau_cmd >= MAX_TORQUE or tau_cmd <= -MAX_TORQUE)
            tau_raw = mrac.calculate_torque(actual_pos, actual_vel, theta_m, omega_m, is_saturated, dt)
            tau_cmd = np.clip(tau_raw, -MAX_TORQUE, MAX_TORQUE)

            # 5. HARDWARE POSITION OVERRIDE (Failsafe)
            if actual_pos >= MAX_SAFE_ANGLE and tau_cmd > 0:
                tau_cmd = 0.0 # Stop pushing further positive
            elif actual_pos <= MIN_SAFE_ANGLE and tau_cmd < 0:
                tau_cmd = 0.0 # Stop pushing further negative

            # 6. COMMAND CURRENT
            current_cmd = tau_cmd / TORQUE_CONSTANT_NM_PER_A
            set_current(ser, current_cmd)

            # 7. LOOP RATE DIAGNOSTICS
            loop_count += 1
            if loop_count % 500 == 0:
                elapsed_total = time.perf_counter() - rate_check_start
                achieved_hz = loop_count / elapsed_total
                print(f"Achieved rate: {achieved_hz:.1f} Hz")

    except KeyboardInterrupt:
        print("\nCtrl+C detected.")

    finally:
        print("Zeroing current and closing port...")
        set_current(ser, 0.0)
        time.sleep(0.05)
        ser.close()
        print("System shutdown safely.")

if __name__ == "__main__":
    main()