import numpy as np
import matplotlib.pyplot as plt

class AK80_6_Plant:
    def __init__(self, dt):
        self.theta = 0.0
        self.omega = 0.0
        self.dt = dt
        
    def step(self, tau_cmd):
        # 1. Hardware Safety Limit (Cap torque at +/- 8 Nm)
        tau_actual = np.clip(tau_cmd, -8.0, 8.0)
        
        # 2. Realistic Mechanical Dynamics
        J = 0.015
        b = 0.1    # Viscous damping (friction that increases with speed)
        
        # Mild kinetic friction (simulates the internal gears/bearings)
        coulomb_friction = 0.3 if self.omega > 0.01 else (-0.3 if self.omega < -0.01 else 0.0)
        
        # 3. Calculate acceleration: sum of torques / inertia
        omega_dot = (tau_actual - (b * self.omega) - coulomb_friction) / J
        
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
        alpha = self.w * self.w * (r - self.theta) - 2.0 * self.z * self.w * self.omega
        self.omega += alpha * self.dt
        self.theta += self.omega * self.dt
        return self.theta, self.omega

class AdmittanceController:
    def __init__(self, Mv, Bv, Kv, dt):
        self.Mv = Mv  
        self.Bv = Bv  
        self.Kv = Kv  
        self.dt = dt
        self.r_react = 0.0      
        self.r_react_dot = 0.0  

    def step(self, tau_human, r_fixed):
        r_react_ddot = (tau_human - (self.Bv * self.r_react_dot) - (self.Kv * (self.r_react - r_fixed))) / self.Mv
        self.r_react_dot += r_react_ddot * self.dt
        self.r_react += self.r_react_dot * self.dt
        return self.r_react
    
class mrac_control:
    def __init__(self, ag, sigma, dt):
        self.ag = ag        
        self.sigma = sigma  
        self.dt = dt

        # Initial baseline gains
        self.Kp = 15.0
        self.Kd = 1.0
        self.Kr = 15.0

    def calculate_torque(self, r, theta_actual, omega_actual, theta_model, saturated):
        e = theta_actual - theta_model
        abs_e = abs(e)

        # "e-Modification" leakage to prevent gains from dropping to zero during rest
        if not saturated:
            self.Kr += (-self.ag * e * theta_model - self.sigma * abs_e * self.Kr) * self.dt
            self.Kp += (self.ag * e * theta_actual - self.sigma * abs_e * self.Kp) * self.dt
            self.Kd += (self.ag * e * omega_actual - self.sigma * abs_e * self.Kd) * self.dt

        # Hardware safety boundaries for gains
        self.Kr = np.clip(self.Kr, 0.0, 50.0)
        self.Kp = np.clip(self.Kp, 0.0, 50.0)
        self.Kd = np.clip(self.Kd, 0.1, 5.0)

        # Output Torque Command instead of Voltage
        tau_cmd = (self.Kr * r) - (self.Kp * theta_actual) - (self.Kd * omega_actual)
        return tau_cmd

if __name__ == "__main__":
    dt = 0.005  
    
    motor = AK80_6_Plant(dt=dt)
    ideal_knee = rm(z=1.0, w=10.0, dt=dt) 
    
    admittance = AdmittanceController(Mv=0.25, Bv=2.5, Kv=8.0, dt=dt)
    controller = mrac_control(ag=1.5, sigma=0.2, dt=dt)
    
    r_fixed = 0.785  # 45 degrees
    
    history = {'time': [], 'tau_h': [], 'r_react': [], 'theta_m': [], 'theta_act': [], 'tau_cmd': [], 'Kp': []}
    
    tau_cmd = 0.0
    
    for step in range(600):  # 3 seconds total
        time_sec = step * dt
        
        actual_theta = motor.theta
        actual_omega = motor.omega
        
        # Human push simulation
        if 1.0 <= time_sec <= 2.0:
            tau_human = -4.0 
        else:
            tau_human = 0.0
            
        r_reactive = admittance.step(tau_human, r_fixed)
        theta_m, omega_m = ideal_knee.update(r_reactive)
        
        # MRAC calculates torque. Saturated if trying to push past our 8 Nm safety limit
        is_saturated = (tau_cmd >= 8.0 or tau_cmd <= -8.0)
        tau_raw = controller.calculate_torque(r_reactive, actual_theta, actual_omega, theta_m, is_saturated)
        tau_cmd = np.clip(tau_raw, -8.0, 8.0)
        
        history['time'].append(time_sec)
        history['tau_h'].append(tau_human)
        history['r_react'].append(r_reactive)
        history['theta_m'].append(theta_m)
        history['theta_act'].append(actual_theta)
        history['tau_cmd'].append(tau_cmd)
        history['Kp'].append(controller.Kp)

        motor.step(tau_cmd)
            
    # --- Plotting ---
    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    fig.suptitle('Hardware-Ready Simulation: Admittance + Torque MRAC', fontsize=14, fontweight='bold')

    axs[0].plot(history['time'], history['r_react'], 'b--', label='Yielding Target', alpha=0.7)
    axs[0].plot(history['time'], history['theta_m'], 'g-', linewidth=2, label='Ideal Model')
    axs[0].plot(history['time'], history['theta_act'], 'r:', linewidth=2, label='Actual Motor Position')
    axs[0].set_ylabel('Position (rad)')
    axs[0].legend(loc='lower right')
    axs[0].grid(True, linestyle='--', alpha=0.6)

    axs[1].plot(history['time'], history['tau_h'], 'm-', label='Human Torque (Nm)')
    axs[1].set_ylabel('Human (Nm)', color='m')
    axs[1].tick_params(axis='y', labelcolor='m')

    ax2_tau = axs[1].twinx()
    ax2_tau.plot(history['time'], history['tau_cmd'], 'k-', label='Motor Commanded Torque (Nm)', alpha=0.6)
    ax2_tau.set_ylabel('Motor (Nm)', color='k')
    ax2_tau.tick_params(axis='y', labelcolor='k')
    axs[1].grid(True, linestyle='--', alpha=0.6)

    axs[2].plot(history['time'], history['Kp'], 'c-', linewidth=2, label='MRAC Kp Gain')
    axs[2].set_xlabel('Time (seconds)')
    axs[2].set_ylabel('Gain Value')
    axs[2].legend(loc='upper right')
    axs[2].grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout()
    plt.show()