import numpy as np
import matplotlib.pyplot as plt

# Mock classes to ensure code runs standalone; replace with your actual imports
class AK80_6_Plant:
    def __init__(self, dt):
        self.theta = 0.0
        self.omega = 0.0
        self.i = 0.0
        self.dt = dt
    def step(self, V):
        # Simplified second-order motor dynamics for simulation
        kt = 0.105
        R = 0.5
        J = 0.005
        b = 0.01
        self.i = (V - kt * self.omega) / R
        tau = kt * self.i
        omega_dot = (tau - b * self.omega) / J
        self.omega += omega_dot * self.dt
        self.theta += self.omega * self.dt

class rm:
    def __init__(self, z, w, dt):
        self.z = z
        self.w = w
        self.dt = dt
        self.theta = 0.0
        self.omega = 0.0
    def update(self, r):
        # Standard second-order reference model
        alpha = self.w * self.w * (r - self.theta) - 2.0 * self.z * self.w * self.omega
        self.omega += alpha * self.dt
        self.theta += self.omega * self.dt
        return self.theta, self.omega


class AdmittanceController:
    def __init__(self, Mv, Bv, Kv, dt):
        # TUNED: Represents a smooth, natural-feeling exoskeleton joint
        self.Mv = Mv  # Virtual Inertia
        self.Bv = Bv  # Virtual Damping
        self.Kv = Kv  # Virtual Stiffness
        self.dt = dt
        
        self.r_react = 0.0      
        self.r_react_dot = 0.0  

    def step(self, tau_human, r_fixed):
        # Dynamic equation: Mv*x_ddot + Bv*x_dot + Kv*(x - r_fixed) = tau_human
        r_react_ddot = (tau_human - (self.Bv * self.r_react_dot) - (self.Kv * (self.r_react - r_fixed))) / self.Mv
        self.r_react_dot += r_react_ddot * self.dt
        self.r_react += self.r_react_dot * self.dt
        return self.r_react
    

class mrac_control:
    def __init__(self, ag, sigma, dt):
        self.ag = ag        # Adaptation rate
        self.sigma = sigma  # Sigma leakage factor (prevents parameter drift)
        self.dt = dt

        # Initial stable baseline gains (PD + Feedforward structure)
        self.Kp = 8.0
        self.Kd = 0.5
        self.Kr = 8.0

    def calculate_voltage(self, r, theta_actual, omega_actual, theta_model, saturated):
        # Error tracking (Actual vs Ideal Model)
        e = theta_actual - theta_model

        # Adaptation Laws with Sigma Leakage
        # Anti-windup: Only adapt gains if the actuator is NOT saturated
        if not saturated:
            self.Kr += (-self.ag * e * theta_model - self.sigma * self.Kr) * self.dt
            self.Kp += (self.ag * e * theta_actual - self.sigma * self.Kp) * self.dt
            self.Kd += (self.ag * e * omega_actual - self.sigma * self.Kd) * self.dt

        # Keep gains bounded inside physically meaningful boundaries
        self.Kr = np.clip(self.Kr, 0.0, 30.0)
        self.Kp = np.clip(self.Kp, 0.0, 30.0)
        self.Kd = np.clip(self.Kd, 0.05, 5.0)

        # Control Law calculation
        V = (self.Kr * r) - (self.Kp * theta_actual) - (self.Kd * omega_actual)
        return V


if __name__ == "__main__":
    # Simulation Configuration
    dt = 0.005  # Reduced to 5ms for better integration stability on hardware
    
    motor = AK80_6_Plant(dt=dt)
    ideal_knee = rm(z=1.0, w=10.0, dt=dt)  # w raised to 10 for crisper tracking
    
    # --- TUNED PARAMETERS ---
    # Admittance: Increased mass and damping to smooth out compliance feel
    admittance = AdmittanceController(Mv=0.25, Bv=2.5, Kv=8.0, dt=dt)
    # MRAC: Softened adaptation rate, enabled sigma leakage to stabilize gains
    controller = mrac_control(ag=0.75, sigma=0.1, dt=dt)
    
    r_fixed = 0.785  # Target: 45 degrees (rad)
    
    history = {'time': [], 'tau_h': [], 'r_react': [], 'theta_m': [], 'theta_act': [], 'V_cmd': [], 'Kp': [], 'Kr': [], 'Kd': []}
    
    V_cmd = 0.0
    
    for step in range(600):  # Double steps since dt is halved
        time_sec = step * dt
        
        # 1. Read Hardware State
        actual_theta = motor.theta
        actual_omega = motor.omega
        
        # 2. Simulate Human Interaction Torque
        # Applies a resistive -3.0 Nm push from 1.0s to 2.0s
        if 1.0 <= time_sec <= 2.0:
            tau_human = -3.0 
        else:
            tau_human = 0.0
            
        # 3. Admittance Layer
        r_reactive = admittance.step(tau_human, r_fixed)
        
        # 4. Reference Model Layer
        theta_m, omega_m = ideal_knee.update(r_reactive)
        
        # 5. MRAC Layer with Anti-Windup Logic
        is_saturated = (V_cmd >= 39.0 or V_cmd <= -39.0)
        V_raw = controller.calculate_voltage(r_reactive, actual_theta, actual_omega, theta_m, is_saturated)
        V_cmd = np.clip(V_raw, -39.0, 39.0)
        
        # Log History
        history['time'].append(time_sec)
        history['tau_h'].append(tau_human)
        history['r_react'].append(r_reactive)
        history['theta_m'].append(theta_m)
        history['theta_act'].append(actual_theta)
        history['V_cmd'].append(V_cmd)
        history['Kp'].append(controller.Kp)
        history['Kr'].append(controller.Kr)
        history['Kd'].append(controller.Kd)

        # 6. Actuate
        motor.step(V_cmd)
            
    # --- Plotting Code ---
    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    fig.suptitle('Exoskeleton Control: Admittance + MRAC Performance', fontsize=14, fontweight='bold')

    axs[0].plot(history['time'], history['r_react'], 'b--', label='Reactive Target (Yielding)', alpha=0.7)
    axs[0].plot(history['time'], history['theta_m'], 'g-', linewidth=2, label='Ideal Model Trajectory')
    axs[0].plot(history['time'], history['theta_act'], 'r:', linewidth=2, label='Actual Motor Position')
    axs[0].set_ylabel('Position (rad)')
    axs[0].set_title('Joint Kinematics')
    axs[0].legend(loc='lower right')
    axs[0].grid(True, linestyle='--', alpha=0.6)

    axs[1].plot(history['time'], history['tau_h'], 'm-', label='Human Applied Torque (Nm)')
    axs[1].set_ylabel('Torque (Nm)', color='m')
    axs[1].tick_params(axis='y', labelcolor='m')

    ax2_volts = axs[1].twinx()
    ax2_volts.plot(history['time'], history['V_cmd'], 'k-', label='Commanded Voltage (V)', alpha=0.5)
    ax2_volts.set_ylabel('Voltage (V)', color='k')
    ax2_volts.tick_params(axis='y', labelcolor='k')
    axs[1].set_title('Human Interaction & Actuator Effort')
    axs[1].grid(True, linestyle='--', alpha=0.6)

    axs[2].plot(history['time'], history['Kp'], 'c-', linewidth=2, label='MRAC Kp')
    axs[2].plot(history['time'], history['Kr'], 'y-', linewidth=2, label='MRAC Kr')
    axs[2].plot(history['time'], history['Kd'], 'g-', linewidth=2, label='MRAC Kd')
    axs[2].set_xlabel('Time (seconds)')
    axs[2].set_ylabel('Gain Value')
    axs[2].set_title('Adaptive Gain Evolution (Sigma Leakage & Anti-Windup Active)')
    axs[2].legend(loc='upper right')
    axs[2].grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout()
    plt.show()