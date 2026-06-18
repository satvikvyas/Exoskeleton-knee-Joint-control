import can

# ==========================================
# 1. MOTOR HARDWARE LIMITS (AK80-6 Defaults)
# ==========================================
P_MIN = -12.5
P_MAX = 12.5
V_MIN = -38.0   # AK80-6 Top Speed Limit in MIT Mode
V_MAX = 38.0
T_MIN = -12.0   # AK80-6 Peak Torque Limit
T_MAX = 12.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0

# ==========================================
# 2. CONVERSION MATH FUNCTIONS
# ==========================================
def float_to_uint(x, x_min, x_max, bits):
    """Converts a float into an integer within the motor's bit resolution"""
    span = (1 << bits) - 1
    # Constrain x to the min/max limits
    x = max(min(x, x_max), x_min)
    # Map to integer
    return int((x - x_min) * span / (x_max - x_min))

def uint_to_float(x_int, x_min, x_max, bits):
    """Converts the motor's raw integer reply back into a float"""
    span = (1 << bits) - 1
    return float(x_int) * (x_max - x_min) / span + x_min

# ==========================================
# 3. PACKING THE 8-BYTE COMMAND (PC -> Motor)
# ==========================================
def pack_cmd(p_des, v_des, kp, kd, t_ff):
    """Compresses 5 floats into exactly 8 hexadecimal bytes"""
    
    # Convert floats to integers based on bit limits
    p_int = float_to_uint(p_des, P_MIN, P_MAX, 16)
    v_int = float_to_uint(v_des, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12)
    
    # Pack the integers into 8 bytes using bitwise shifts
    msg_data = bytearray(8)
    msg_data[0] = p_int >> 8
    msg_data[1] = p_int & 0xFF
    msg_data[2] = v_int >> 4
    msg_data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
    msg_data[4] = kp_int & 0xFF
    msg_data[5] = kd_int >> 4
    msg_data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
    msg_data[7] = t_int & 0xFF
    
    return msg_data

# ==========================================
# 4. UNPACKING THE 6-BYTE REPLY (Motor -> PC)
# ==========================================
def unpack_reply(msg_data):
    """Decompresses the motor's 6-byte reply into position, velocity, and current"""
    
    # Extract the integers using bitwise masks
    id_int = msg_data[0]
    p_int = (msg_data[1] << 8) | msg_data[2]
    v_int = (msg_data[3] << 4) | (msg_data[4] >> 4)
    i_int = ((msg_data[4] & 0xF) << 8) | msg_data[5]
    
    # Convert back to actual physics floats
    pos = uint_to_float(p_int, P_MIN, P_MAX, 16)
    vel = uint_to_float(v_int, V_MIN, V_MAX, 12)
    current = uint_to_float(i_int, T_MIN, T_MAX, 12)
    
    return {"id": id_int, "position": pos, "velocity": vel, "current": current}