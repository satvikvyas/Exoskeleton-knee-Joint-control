class pidcontroller:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0.0
        self.prev_e = 0.0

    def update(self, error , dt):
        self.error = error
        self.dt = dt

        #the p term is for proportional system 
        p = self.kp*error
        # the i term is for past errors 
        self.integral += dt*error 
        i = self.ki * self.integral
        #the d term is a correction signal which changes rate if correction from p and i 
        deri = (error - self.prev_e) / dt
        d = deri *  self.kd

        #here kd, ki and kp are three fundamental gain parameters  
        self.prev_e = error

        return p + i + d