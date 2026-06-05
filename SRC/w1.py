import numpy as py
import control as ct
import can
import time

# ==========================================
# 1. PHYSICAL CONSTANTS & CONVERSIONS
# ==========================================
b = 0.02      # Viscous friction (Ns/rad)
j = 607 / 1e7    # Inertia in kg*m^2 (Converted from g*cm^2)
r = 170 / 1000   # Resistance in Ohms (Phase-to-Phase)
ke = 0.10026     # Electrical constant in V/(rad/s) (Converted from 10.5 V/krpm)
kt = 0.0105      # Torque constant in Nm/A
l = 57 / 1e6     # Inductance in H (Phase-to-Phase)

dt = 0.01        # Execution time step (100 Hz / 10 milliseconds)
safe_current = 10.0

# ==========================================
# 2. CONTINUOUS STATE-SPACE THEORY
# ==========================================
A_c = py.array([[-b/j  , kt/j ],
                [-ke/l , -r/l ]])

B_c = py.array([[0],
                [1/l]])

# C matrix: We observe both states (Velocity and Current)
C_c = py.array([[1.0, 0.0],
                [0.0, 1.0]])

# D matrix: No instantaneous feedforward from input to output
D_c = py.array([[0.0],
                [0.0]])

# Assemble the continuous system representation
sys_continuous = ct.ss(A_c, B_c, C_c, D_c)

# ==========================================
# 3. SYSTEM DISCRETIZATION (ZOH)
# ==========================================
# Convert the smooth continuous physics into digital time steps (dt = 0.01)
sys_discrete = ct.sample_system(sys_continuous, dt, method='zoh')
A_d = sys_discrete.A
B_d = sys_discrete.B

print("--- SYSTEM MODEL READY ---")
print(f"Discrete-Time State Matrix A_d:\n{A_d}")
print(f"Discrete-Time Input Matrix B_d:\n{B_d}\n--------------------------")


# 4. MANUAL PID GAIN CONFIGURATION

#to be tuned
Kp = 5.0    # Proportional Gain: 
Ki = 0.1    # Integral Gain: Eliminates steady-state droop over time
Kd = 0.2    # Derivative Gain: Damps out oscillations/vibrations

# ==========================================
# 5. HARDWARE UTILITY FUNCTIONS
# ==========================================


def feedback(data):
    """Parses raw CAN frames from the AK80-6"""
    # Position decoding (16-bit)
    raw_position = (data[1] << 8) | data[2]
    position_rad = -12.5 + raw_position * (25.0) / 65535.0
    
    # Current decoding (12-bit)
    raw_current = (data[3] << 4) | (data[4] >> 4)
    current_amps = raw_current # Keep raw for initialization diagnostics
    
    torque = 0.0
    return position_rad, current_amps, torque


def read_user_effort():
    """Simulates or reads human intent/force applied to the exoskeleton leg"""
    # For baseline testing, assume the human is applying 0 external effort
    return 0.0

def bus_connection():
    """Initializes the communication pathway on COM5"""
    try: 
        bus = can.interface.Bus(interface='slcan', channel='COM5', bitrate=1000000, tty_baudrate=921600)
        print("CAN communication channel active on COM5.")
        return bus
    except Exception as e :
        print(f"Port allocation failed: {e}")
        return None

# ==========================================
# 6. MAIN HIGH-FREQUENCY DISCRETE LOOP

bus = bus_connection()


if bus is not None:
    # Initialize persistent state variables for discrete PID integration/derivation
    integral_error = 0.0
    last_position_error = 0.0
    
    try:
        print("Starting Manual PID Control Loop. Standing by for telemetry...")
        while True:
            start_time = time.time() # Frame timestamp marker
            
            # Non-blocking check for incoming hardware frames
            msg = bus.recv(timeout = 0.005)
            user_effort = read_user_effort()
            
            
            if msg is not None: 
                # Unpack the current physical state of the joint
                actual_pos, actual_current, _ = feedback(msg.data)
                
                # --- ACTIVE SAFETY GUARDRAIL ---
                if actual_current > safe_current:
                    print(f"[EMERGENCY SHUTDOWN] Current spiked to {actual_current}A. Exiting.")
                    break
                
                # --- ADAPTIVE TARGET TRAJECTORY ---
                # Human interaction effort dynamically offsets the baseline target angle
                baseline_target_angle = 0.0 # Standard standing position
                target_pos = baseline_target_angle + (user_effort * 0.05) 
                
                # --- DISCRETE PID CALCULATIONS ---
                # 1. Error calculation
                pos_error = target_pos - actual_pos
                
                # 2. Proportional term: Instantaneous correction
                p_term = Kp * pos_error
                
                # 3. Integral term: Accumulate error over discrete time steps (dt)
                integral_error = integral_error + (pos_error * dt)
                # Anti-windup clamp: Prevent the integral term from building up infinitely
                integral_error = max(min(integral_error, 5.0), -5.0)
                i_term = Ki * integral_error
                
                # 4. Derivative term: Rate of change of error between discrete steps
                derivative_error = (pos_error - last_position_error) / dt
                source_damping = Kd * derivative_error
                
                # Total computed control execution command
                commanded_torque = p_term + i_term + source_damping
                
                # Log state to terminal for analysis
                print(f"Target: {target_pos:.3f} | Actual: {actual_pos:.3f} | Output Torque: {commanded_torque:.3f}Nm")
                
                # Save error state for the next discrete loop step (k + 1)
                last_position_error = pos_error
                
            # Enforce exact, predictable 100 Hz timing cycle
            elapsed_time = time.time() - start_time
            if elapsed_time < dt:
                time.sleep(dt - elapsed_time)

    except KeyboardInterrupt:
        print('\nControl loop deactivated by user.')

    finally:
        bus.shutdown()
        print("CAN interface released safely.")
else:
    print("Execution halted: connection unavailable.")