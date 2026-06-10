import numpy as py
import control as ct
import matplotlib.pyplot as plt
from pid import pidcontroller as pid
import math


# 1. PHYSICAL CONSTANTS & CONVERSIONS

b = 0.02          # Viscous friction (Ns/rad)
j = 607 / 1e7     # Motor Inertia in kg*m^2 
r = 12.78 / 1000  # Resistance in Ohms 
ke = 0.10026      # Electrical constant in V/(rad/s) 
kt = 0.10026      # Torque constant in Nm/A (FIXED: Must match ke)
l = 15.3 / 1e6    # Inductance in H  

dt = 0.001         # Execution time step 
safe_current = 10.0
battery_voltage = 40.0


# 2. HUMAN & GEARBOX PARAMETERS

user_effort = 5.0 # Nm (FIXED: Reduced to a realistic casual leg swing force)
set_points = [3.14/2] # Sitting position is assumed to be 0 

leg_length = 0.25 # in metres 
m = 4.0           # Lower leg mass in kg

N = 50.0          # Gear ratio 

# Calculate Effective Inertia felt by the motor
j_load = m * (leg_length ** 2)
j_eff = j + (j_load / (N ** 2))

# # ==========================================
# # 3. STATE-SPACE MATRICES
# # ==========================================
# # FIXED: Using j_eff instead of j

# A_c = py.array([[0, 1 , 0],
#                 [0, -b/j_eff, kt/j_eff],
#                 [0, -ke/l, -r/l]])

# B_c = py.array([[0],   
#                 [0],
#                 [1/l]])

# C_c = py.array([[1.0, 0.0, 0.0],
#                 [0.0, 1.0, 0.0],
#                 [0.0, 0.0, 1.0]])

# D_c = py.array([[0], [0], [0]])

# B_dist = py.array([[0],
#                    [1/j_eff],
#                    [0]])

# sys_c = ct.ss(A_c, B_c, C_c, D_c)
# sys_d = sys_c.sample(dt, method='zoh')
# B_dd = B_dist * dt

# ==========================================
# 4. INITIALIZATION
# ==========================================
x = py.array([[0.0], [0.0], [0.0]]) # pos, omega, current



# PID controller 
pids = pid(kp=2.0, kd=0.5, ki=0.0)



history_t = []
history_current = []
history_joint_pos = [] # We want to plot the joint angle, not the motor angle

simulation_time = 10.0 
steps = int(simulation_time / dt)

# ==========================================
# 5. SIMULATION LOOP
# ==========================================
print("Starting simulation...")

for step in range(steps):
    current_time = step * dt
    
    # 1. Kinematics: Convert motor position to joint position
    motor_pos = x[0].item()
    joint_pos = motor_pos / N
    
    # 2. Calculate Disturbance Torques
    # Gravity acts on the joint
    t_grav_joint = m * 9.81 * leg_length * math.sin(joint_pos)
    
    # Net external torque acting on the joint
    tau_load_joint = user_effort - t_grav_joint 
    
    # Torque felt by the motor (divided by gear ratio)
    tau_load_motor = tau_load_joint / N
    
    # 3. Control Logic (PID)
    motor_setpoint = set_points[0] * N
    e = motor_setpoint - motor_pos
    v_cmd = pids.update(e, dt) 
    
    # 4. Hardware Safety Limits (VESC/ODrive Logic)
    # Calculate current Back-EMF: e_b = Ke * omega
    motor_velocity = x[1].item()
    back_emf = ke * motor_velocity
    
    # Calculate max/min voltage allowed to keep current under 10A
    # V_limit = e_b + (I_max * R)
    v_max_for_current = back_emf + (safe_current * r)
    v_min_for_current = back_emf - (safe_current * r)
    
    # Clamp PID output to safe current voltage limits FIRST
    v_cmd = max(v_min_for_current, min(v_max_for_current, v_cmd))
    
    # Clamp to actual physical battery voltage LAST
    v_cmd = max(-battery_voltage, min(battery_voltage, v_cmd))

    # 5. Physics Engine Update
    x = sys_d.A @ x + sys_d.B * v_cmd + B_dd * tau_load_motor
    # 5. Physics Engine Update
    x = sys_d.A @ x + sys_d.B * v_cmd + B_dd * tau_load_motor
    
    # 6. Log data for plotting
    history_t.append(current_time)
    history_joint_pos.append(joint_pos)
    history_current.append(x[2].item())


# 6. PLOTTING


plt.figure(figsize=(10, 8))

# Plot Position (Joint Angle)
plt.subplot(2, 1, 1)
plt.plot(history_t, history_joint_pos, label='Actual Joint Position (rad)', color='blue')
plt.axhline(y=set_points[0], color='r', linestyle='--', label='Target Joint Setpoint')
plt.title('Exoskeleton Knee Joint - Position Response (Geared 50:1)')
plt.ylabel('Angle (rad)')
plt.grid(True)
plt.legend()

# Plot Current
plt.subplot(2, 1, 2)
plt.plot(history_t, history_current, label='Motor Current (A)', color='orange')
plt.axhline(y=safe_current, color='red', linestyle=':', label='Safe Current Limit (10A)')
plt.axhline(y=-safe_current, color='red', linestyle=':')
plt.title('Motor Current Draw')
plt.xlabel('Time (seconds)')
plt.ylabel('Current (A)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()