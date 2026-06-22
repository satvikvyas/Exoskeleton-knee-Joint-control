import numpy as np

class AK80_6_Plant:
    def __init__(self, dt=0.01):
        self.J = 0.0000607
        self.R = 0.170
        self.L = 0.000057
        self.Kt = 0.105
        self.Ke = 0.10026
        self.b = 0.02
        
        # Outer control loop timing (100 Hz)
        self.dt = dt
        
        # Inner physics loop timing (10,000 Hz)
        self.sim_steps = 100  
        self.sim_dt = self.dt / self.sim_steps 
        
        self.theta = 0.0
        self.omega = 0.0
        self.i = 0.0

    def step(self, V_command):
        # Run 100 micro-steps of physics for every 1 control command
        for _ in range(self.sim_steps):
            i_dot = -(self.Ke / self.L) * self.omega - (self.R / self.L) * self.i + (1 / self.L) * V_command
            omega_dot = -(self.b / self.J) * self.omega + (self.Kt / self.J) * self.i
            theta_dot = self.omega

            # Step forward using the tiny micro-timestep (0.1 ms)
            self.i += i_dot * self.sim_dt
            self.omega += omega_dot * self.sim_dt
            self.theta += theta_dot * self.sim_dt
        
        self.i = np.clip(self.i, -10.0, 10.0)
        return self.theta, self.omega, self.i