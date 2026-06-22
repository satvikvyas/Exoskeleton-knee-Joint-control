import numpy as np
from reference_model_mrac import referencem as rm
from ak80_6 import AK80_6_Plant
import matplotlib.pyplot as plt

class AdmittanceController:
    def __init__(self, Mv=0.05, Bv=0.5, Kv=5.0, dt=0.01):
        # Tuning Parameters (The "Feel" of the exoskeleton)
        self.Mv = Mv  # Virtual Mass (Lower = yields faster to human)
        self.Bv = Bv  # Virtual Damping (Higher = prevents jerky yielding)
        self.Kv = Kv  # Virtual Stiffness (Higher = fights harder to return to setpoint)
        self.dt = dt
        
        self.r_react = 0.0      # Yielding target position
        self.r_react_dot = 0.0  # Yielding target velocity

    def step(self, tau_human, r_fixed):
        """
        Calculates the yielding setpoint based on applied human force.
        """
        r_react_ddot = (tau_human - (self.Bv * self.r_react_dot) - (self.Kv * (self.r_react - r_fixed))) / self.Mv
        self.r_react_dot += r_react_ddot * self.dt
        self.r_react += self.r_react_dot * self.dt
        return self.r_react
    

class mrac_control:
    def __init__(self , ag , sigma , dt):
        self.ag= ag
        self.sigma = sigma
        self.dt = dt

        #starting the gains at 0
        self.Kp= 0.0
        self.Kd= 0.0
        self.Kr= 0.0

    def calculate_voltage(self,r, theta_actual ,omega_actual , theta_model):
        

        e = theta_actual - theta_model

        self.Kr += (-self.ag * e * theta_model - self.sigma * self.Kr) * self.dt
        self.Kp += (self.ag * e * theta_actual - self.sigma * self.Kp) * self.dt
        self.Kd += (self.ag * e * omega_actual - self.sigma * self.Kd) * self.dt

        V = (self.Kr * r) - (self.Kp * theta_actual) - (self.Kd * omega_actual)

        return np.clip(V, -39.0, 39.0)


if __name__ == "__main__":
    # 1. Initialize External Modules
    motor = AK80_6_Plant(dt=0.01)
    ideal_knee = rm(z=1.0, w=10.0, dt=0.01)
    
    # 2. Initialize Internal Controllers
    admittance = AdmittanceController(Mv=0.05, Bv=0.5, Kv=5.0, dt=0.01)
    controller = mrac_control(ag=5.0, sigma=0.05, dt=0.01)
    
    # 3. System Constants
    r_fixed = 0.785      # Predetermined target (45 degrees)
    Kt = 0.105           # AK80-6 Torque Constant (Nm/A)

    r_fixed = 0.785  # 45 degrees
    Kt = 0.105
    
    # Data storage for plotting
    history = {'time': [], 'tau_h': [], 'r_react': [], 'theta_m': [], 'theta_act': [], 'V_cmd': [], 'Kp': []}
    
       
    # Simulate 1.5 seconds of operation (150 steps at 100 Hz)
    for step in range(150):
        # --------------------------------------------------
        # PIPELINE STEP 1: Read Hardware State
        # --------------------------------------------------
        # In reality, this data comes unpacked from the CAN Bus
        actual_theta = motor.theta
        actual_omega = motor.omega
        actual_i = motor.i
        
        # --------------------------------------------------
        # PIPELINE STEP 2: Estimate Human Interaction
        # --------------------------------------------------
        # SIMULATION INJECTION: Human applies 2 Nm of resistance between 0.5s and 1.0s
        time_sec = step * 0.01
        if 0.5 <= time_sec <= 1.0:
            tau_human = -2.0 
        else:
            # Baseline estimation from phase current
            tau_human = -(Kt * actual_i) 
            
        # --------------------------------------------------
        # PIPELINE STEP 3: Admittance Layer
        # --------------------------------------------------
        r_reactive = admittance.step(tau_human, r_fixed)
        
        # --------------------------------------------------
        # PIPELINE STEP 4: Reference Model Layer
        # --------------------------------------------------
        theta_m, omega_m = ideal_knee.update(r_reactive)
        
        # --------------------------------------------------
        # PIPELINE STEP 5: MRAC Layer
        # --------------------------------------------------
        V_cmd = controller.calculate_voltage(r_reactive, actual_theta, actual_omega, theta_m)
        

        history['time'].append(time_sec)
        history['tau_h'].append(tau_human)
        history['r_react'].append(r_reactive)
        history['theta_m'].append(theta_m)
        history['theta_act'].append(actual_theta)
        history['V_cmd'].append(V_cmd)
        history['Kp'].append(controller.Kp)

    
        # In reality, this is bit-packed and sent over SocketCAN
        motor.step(V_cmd)
            
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

        # Subplot 3: Adaptation (Gains)
    axs[2].plot(history['time'], history['Kp'], 'c-', linewidth=2, label='MRAC Kp Gain')
    axs[2].set_xlabel('Time (seconds)')
    axs[2].set_ylabel('Gain Value')
    axs[2].set_title('Adaptive Gain Evolution (Sigma Leakage Active)')
    axs[2].legend(loc='upper right')
    axs[2].grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout()
    plt.show()