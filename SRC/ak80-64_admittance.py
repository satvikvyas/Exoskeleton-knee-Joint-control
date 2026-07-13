import struct
import time
import math
import serial

# ==========================================
# 1. HARDWARE CONSTANTS
# ==========================================
SERIAL_PORT = 'COM6'
BAUD_RATE = 921600
POLE_PAIRS = 21       # confirmed
GEAR_RATIO = 64.0     # confirmed

# --- TODO / CONFIRM before running ---
TORQUE_CONSTANT_NM_PER_A = None  # <-- MUST SET (Nm per Amp of phase current)

DT = 0.01          # 100 Hz -- matches your validated telemetry rate
MAX_TORQUE = 8.0    # Nm, hardware safety limit on commanded torque

# ==========================================
# 2. SAFETY LIMITS (joint range: duck-sit to standing)
# ==========================================
POS_MIN_DEG = -160.0
POS_MAX_DEG = 0.0
POS_MIN_RAD = math.radians(POS_MIN_DEG)
POS_MAX_RAD = math.radians(POS_MAX_DEG)

MAX_MISSED_READS = 10        # consecutive telemetry misses before zeroing current
MAX_TORQUE_SLEW_PER_LOOP = 2.0  # Nm change allowed per loop -- limits sudden jumps from a bad estimate

# ==========================================
# 3. SERVO-MODE UART PROTOCOL
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

def get_state(ser: serial.Serial):
    """Returns (position_deg, velocity_rad_s, motor_current_a) or None on miss."""
    reply = send_command(ser, bytes([COMM_GET_VALUES]))
    if reply is None or len(reply) < 58:
        return None

    motor_current = struct.unpack('>i', reply[5:9])[0] / 100.0
    erpm = struct.unpack('>i', reply[23:27])[0]
    pos_deg = struct.unpack('>i', reply[54:58])[0] / 1_000_000.0

    motor_rpm = erpm / POLE_PAIRS
    motor_rad_s = motor_rpm * (2 * math.pi / 60.0)
    velocity_rad_s = motor_rad_s / GEAR_RATIO

    return pos_deg, velocity_rad_s, motor_current

# ==========================================
# 4. ADMITTANCE MODEL
# ==========================================
class AdmittanceModel:
    """Mv*a + Bv*v + Kv*(x - r) = tau_ext. Operates in radians internally."""
    def __init__(self, Mv, Bv, Kv, dt, pos_min_rad, pos_max_rad):
        self.Mv = Mv
        self.Bv = Bv
        self.Kv = Kv
        self.dt = dt
        self.pos_min = pos_min_rad
        self.pos_max = pos_max_rad
        self.theta_m = 0.0
        self.omega_m = 0.0

    def step(self, tau_ext, r_baseline):
        alpha = (tau_ext - (self.Bv * self.omega_m) - (self.Kv * (self.theta_m - r_baseline))) / self.Mv
        self.omega_m += alpha * self.dt
        self.theta_m += self.omega_m * self.dt

        if self.theta_m > self.pos_max:
            self.theta_m = self.pos_max
            if self.omega_m > 0:
                self.omega_m = 0.0
        elif self.theta_m < self.pos_min:
            self.theta_m = self.pos_min
            if self.omega_m < 0:
                self.omega_m = 0.0

        return self.theta_m, self.omega_m

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.val = 0.0
    def update(self, new_val):
        self.val = (self.alpha * new_val) + ((1.0 - self.alpha) * self.val)
        return self.val

# ==========================================
# 5. MAIN LOOP
# ==========================================
def main():
    if TORQUE_CONSTANT_NM_PER_A is None:
        print("ERROR: TORQUE_CONSTANT_NM_PER_A must be set before running.")
        return

    print("Opening serial port...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        time.sleep(0.2)
    except Exception as e:
        print(f"Failed to open port: {e}")
        return

    # Admittance tuning -- starting defaults, will need bench tuning
    Mv = 0.25
    Bv = 2.5
    Kv = 8.0
    r_baseline_deg = 0.0  # neutral = standing

    adm = AdmittanceModel(Mv, Bv, Kv, DT, POS_MIN_RAD, POS_MAX_RAD)
    current_filter = LowPassFilter(alpha=0.3)

    last_tau_cmd = 0.0
    missed_reads = 0
    loop_count = 0
    rate_check_start = time.perf_counter()

    print(f"Admittance control active (current-mode). Range clamped to [{POS_MIN_DEG}, {POS_MAX_DEG}] deg.")
    print("Push on the joint to test compliance. Press Ctrl+C to stop.")

    try:
        while True:
            loop_start = time.perf_counter()

            state = get_state(ser)
            if state is not None:
                missed_reads = 0
                actual_pos_deg, actual_vel_rad_s, motor_current = state
                filtered_current = current_filter.update(motor_current)
                actual_torque = filtered_current * TORQUE_CONSTANT_NM_PER_A

                tau_friction = 0.05 * actual_vel_rad_s
                tau_ext_est = -actual_torque - tau_friction

                theta_m, omega_m = adm.step(tau_ext_est, math.radians(r_baseline_deg))

                # Admittance output becomes the torque command directly --
                # no inner position loop to fight, unlike COMM_SET_POS.
                e_pos = math.radians(actual_pos_deg) - theta_m
                e_vel = actual_vel_rad_s - omega_m
                Kp_track, Kd_track = 15.0, 1.0  # fixed tracking gains (not adaptive)
                tau_raw = -(Kp_track * e_pos) - (Kd_track * e_vel)

                # Torque slew limit
                delta = tau_raw - last_tau_cmd
                if delta > MAX_TORQUE_SLEW_PER_LOOP:
                    tau_raw = last_tau_cmd + MAX_TORQUE_SLEW_PER_LOOP
                elif delta < -MAX_TORQUE_SLEW_PER_LOOP:
                    tau_raw = last_tau_cmd - MAX_TORQUE_SLEW_PER_LOOP

                tau_cmd = max(-MAX_TORQUE, min(MAX_TORQUE, tau_raw))
                last_tau_cmd = tau_cmd

                current_cmd = tau_cmd / TORQUE_CONSTANT_NM_PER_A
                set_current(ser, current_cmd)

                print(f"\rPos: {actual_pos_deg:7.2f} deg | Tau_ext: {actual_torque:6.2f} Nm | "
                      f"Tau_cmd: {tau_cmd:6.2f} Nm | Virtual: {math.degrees(theta_m):7.2f} deg", end="")
            else:
                missed_reads += 1
                if missed_reads >= MAX_MISSED_READS:
                    set_current(ser, 0.0)
                    last_tau_cmd = 0.0
                    print(f"\rWARNING: {missed_reads} missed reads -- current zeroed", end="")

            loop_count += 1
            if loop_count % 500 == 0:
                elapsed_total = time.perf_counter() - rate_check_start
                print(f"\n[Achieved rate: {loop_count/elapsed_total:.1f} Hz]")

            elapsed = time.perf_counter() - loop_start
            sleep_time = DT - elapsed
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