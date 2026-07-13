import can
import time
import math

# ==========================================
# 1. PHYSICAL & VIRTUAL CONSTANTS
# ==========================================
DT = 0.001              
M_LEG = 4.0             
L_LEG = 0.25            
G = 9.81                

# Bench-Test Admittance Parameters
M_VIRTUAL = 0.05        
B_VIRTUAL = 0.2         
K_VIRTUAL = 2.0         

# Hardware Safety Limits (Software Stops)
MAX_SAFE_ANGLE = math.radians(45)  # Max positive rotation
MIN_SAFE_ANGLE = math.radians(-45) # Max negative rotation
MAX_SAFE_VEL = 10.0                # rad/s

# T-Motor AK80-6 MIT Mode Limits
P_MIN = -12.5; P_MAX = 12.5
V_MIN = -65.0; V_MAX = 65.0
T_MIN = -18.0; T_MAX = 18.0
KP_MIN = 0.0;  KP_MAX = 500.0
KD_MIN = 0.0;  KD_MAX = 5.0

# ==========================================
# 2. HELPER FUNCTIONS (CAN & MATH)
# ==========================================
def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    x = max(min(x, x_max), x_min)
    return int(((x - x_min) * ((1 << bits) - 1)) / span)

def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    return ((float(x_int) * span) / float((1 << bits) - 1)) + x_min

def pack_mit_command(p_des, v_des, kp, kd, t_ff):
    p_int = float_to_uint(p_des, P_MIN, P_MAX, 16)
    v_int = float_to_uint(v_des, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12)

    data = bytearray(8)
    data[0] = p_int >> 8
    data[1] = p_int & 0xFF
    data[2] = v_int >> 4
    data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
    data[4] = kp_int & 0xFF
    data[5] = kd_int >> 4
    data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
    data[7] = t_int & 0xFF
    return data

def unpack_motor_reply(data):
    if len(data) < 6:
        return 0.0, 0.0, 0.0
    
    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    t_int = ((data[4] & 0xF) << 8) | data[5]

    return (uint_to_float(p_int, P_MIN, P_MAX, 16),
            uint_to_float(v_int, V_MIN, V_MAX, 12),
            uint_to_float(t_int, T_MIN, T_MAX, 12))

# ==========================================
# 3. CONTROLLER CLASSES
# ==========================================
class AdmittanceController:
    def __init__(self, Mv, Bv, Kv, dt):
        self.Mv = Mv
        self.Bv = Bv
        self.Kv = Kv
        self.dt = dt
        self.theta_v = 0.0      
        self.theta_dot_v = 0.0  

    def step(self, tau_human, theta_ref):
        displacement = self.theta_v - theta_ref
        theta_ddot_v = (tau_human - (self.Bv * self.theta_dot_v) - (self.Kv * displacement)) / self.Mv
        self.theta_dot_v += theta_ddot_v * self.dt
        self.theta_v += self.theta_dot_v * self.dt
        return self.theta_v, self.theta_dot_v

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
    print("Initializing CAN interface...")
    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
    except Exception as e:
        print(f"Failed to open CAN bus: {e}")
        return

    motor_id = 0x01 
    admittance = AdmittanceController(M_VIRTUAL, B_VIRTUAL, K_VIRTUAL, DT)
    
    # Initialize EMA Filters (Alpha = 0.15 is a good starting point for 1000Hz)
    vel_filter = LowPassFilter(alpha=0.15)
    accel_filter = LowPassFilter(alpha=0.10)
    
    theta_baseline = 0.0 

    try:
        print("Entering Motor Mode...")
        enter_cmd = can.Message(arbitration_id=motor_id, data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], is_extended_id=False)
        bus.send(enter_cmd)
        time.sleep(0.1)

        print("Starting 1000Hz Loop. Press Ctrl+C to stop.")
        
        actual_pos = 0.0
        prev_vel_filtered = 0.0
        
        while True:
            loop_start = time.perf_counter()

            # 1. READ & FILTER SENSOR DATA
            req_msg = can.Message(arbitration_id=motor_id, data=pack_mit_command(0, 0, 0, 0, 0), is_extended_id=False)
            bus.send(req_msg)
            
            reply = bus.recv(timeout=0.0005)
            if reply is not None:
                raw_pos, raw_vel, actual_torque = unpack_motor_reply(reply.data)
                actual_pos = raw_pos
                actual_vel_filtered = vel_filter.update(raw_vel)
            else:
                actual_vel_filtered = prev_vel_filtered # Fallback if packet drops

            # 2. INVERSE DYNAMICS (Bench Test Configuration)
            raw_accel = (actual_vel_filtered - prev_vel_filtered) / DT
            actual_accel_filtered = accel_filter.update(raw_accel)
            prev_vel_filtered = actual_vel_filtered
            
            tau_gravity = 0.0 # Disabled for bench test
            tau_friction = 0.05 * actual_vel_filtered 
            
            tau_human_est = actual_torque - tau_gravity - tau_friction

            # 3. ADMITTANCE MATH
            raw_target_pos, raw_target_vel = admittance.step(tau_human_est, theta_baseline)

            # 4. SAFETY CLAMPING
            target_pos = max(min(raw_target_pos, MAX_SAFE_ANGLE), MIN_SAFE_ANGLE)
            target_vel = max(min(raw_target_vel, MAX_SAFE_VEL), -MAX_SAFE_VEL)

            # 5. SEND COMMAND
            inner_kp = 25.0  
            inner_kd = 0.5   
            cmd_data = pack_mit_command(target_pos, target_vel, inner_kp, inner_kd, 0.0)
            msg = can.Message(arbitration_id=motor_id, data=cmd_data, is_extended_id=False)
            bus.send(msg)

            # 6. HYBRID PRECISION PACING
            # Sleep to yield CPU, but wake up slightly early (200 microseconds)
            elapsed = time.perf_counter() - loop_start
            sleep_time = DT - elapsed
            if sleep_time > 0.0002:
                time.sleep(sleep_time - 0.00015) 
            
            # Spin-wait the remaining microseconds for exact precision
            while (time.perf_counter() - loop_start) < DT:
                pass

    except KeyboardInterrupt:
        print("\nCtrl+C detected.")
        
    finally:
        print("Disabling Motor and Exiting...")
        exit_cmd = can.Message(arbitration_id=motor_id, data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD], is_extended_id=False)
        bus.send(exit_cmd)
        bus.shutdown()
        print("System shutdown safely.")

if __name__ == "__main__":
    main()