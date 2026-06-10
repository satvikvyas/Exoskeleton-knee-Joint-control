import numpy as py
import control as ct
import matplotlib.pyplot as plt
from pid import pidcontroller as pid
import math

# ==========================================
# 1. PHYSICAL CONSTANTS & CONVERSIONS
# ==========================================
b = 0.02          # Viscous friction (Ns/rad)
j = 607 / 1e7     # Motor Inertia in kg*m^2 
r = 12.78 / 1000  # Resistance in Ohms 
ke = 0.10026      # Electrical constant in V/(rad/s) 
kt = 0.10026      # Torque constant in Nm/A
l = 15.3 / 1e6    # Inductance in H  

dt = 0.001        # Execution time step (1 kHz / 1 millisecond)
safe_current = 10.0
battery_voltage = 40.0

# ==========================================
# 2. HUMAN & GEARBOX PARAMETERS
# ==========================================
user_effort = 5.0 # Nm (Casual leg swing force)
set_points = [-3.14/4, 0, 3.14/2] # Target angles in radians

leg_length = 0.25 # in metres 
m = 4.0           # Lower leg mass in kg
N = 50.0          # Gear ratio (50:1)

# Calculate Effective Inertia felt by the motor
j_load = m * (leg_length ** 2)
j_eff = j + (j_load / (N ** 2))

# ==========================================
# 3. STATE-SPACE MATRICES
# ==========================================
A_c = py.array([[0, 1 , 0],
                [0, -b/j_eff, kt/j_eff],
                [0, -ke/l, -r/l]])

B_c = py.array([[0],   
                [0],
                [1/l]])

C_c = py.array([[1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]])

D_c = py.array([[0], [0], [0]])

B_dist = py.array([[0],
                   [1/j_eff],
                   [0]])

sys_c = ct.ss(A_c, B_c, C_c, D_c)
sys_d = sys_c.sample(dt, method='zoh')
B_dd = B_dist * dt

# ==========================================
# 4. INITIALIZATION (CASCADE PID SETUP)
# ==========================================
x = py.array([[0.0], [0.0], [0.0]]) # [pos, omega, current]

# OUTER LOOP: Position Controller (Outputs Amps)

pos_pid = pid(kp=50.0, kd=0.5, ki=5.0) 

# INNER LOOP: Current Controller (Outputs Volts)
# Current loops use PI, heavily relying on Proportional gain
current_pi = pid(kp=20.0, kd=0.0, ki=1.0)

history_t = []
history_current = []
history_joint_pos = [] 

simulation_time = 10.0 
steps = int(simulation_time / dt)

# 5. SIMULATION LOOP

print("Starting Cascade Control Simulation...")

for step in range(steps):
    current_time = step * dt
    
    # Extract current states
    motor_pos = x[0].item()
    motor_vel = x[1].item()
    actual_current = x[2].item()
    
    # 1. Kinematics & Disturbances
    joint_pos = motor_pos / N
    t_grav_joint = m * 9.81 * leg_length * math.sin(joint_pos)
    tau_load_joint = user_effort - t_grav_joint 
    tau_load_motor = tau_load_joint / N
    
    # 2. OUTER LOOP
    
    motor_setpoint = set_points[2] * N  # Testing the 90-degree (pi/2) lift
    pos_error = motor_setpoint - motor_pos
    
    target_current = pos_pid.update(pos_error, dt) 
    
    # HARD LIMIT: Cap the requested current to hardware safety limits
    target_current = max(-safe_current, min(safe_current, target_current))
    
    # ----------------------------------------------------
    # 3. INNER LOOP (Current -> Voltage)
    # ----------------------------------------------------
    current_error = target_current - actual_current
    
    v_cmd = current_pi.update(current_error, dt)
    
    # Feedforward: Add the Back-EMF to help the PI controller
    back_emf = ke * motor_vel
    v_cmd += back_emf
    
    # HARD LIMIT: Cap voltage to battery specs
    v_cmd = max(-battery_voltage, min(battery_voltage, v_cmd))

    # ----------------------------------------------------
    # 4. Physics Engine Update
    # ----------------------------------------------------
    x = sys_d.A @ x + sys_d.B * v_cmd + B_dd * tau_load_motor
    
    # 5. Log data
    history_t.append(current_time)
    history_joint_pos.append(joint_pos)
    history_current.append(actual_current)

# ==========================================
# 6. PLOTTING RESULTS
# ==========================================
plt.figure(figsize=(10, 8))

# Plot Position (Joint Angle)
plt.subplot(2, 1, 1)
plt.plot(history_t, history_joint_pos, label='Actual Joint Position (rad)', color='blue')
plt.axhline(y=set_points[2], color='r', linestyle='--', label='Target Joint Setpoint')
plt.title('Cascade Control - Position Response')
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