class referencem:
    def __init__(self, z , w , dt):
        self.z = z 
        self.w = w
        self.dt= dt

        self.theta_p = 0.0
        self.theta_v = 0.0

    def update(self, r):
     #step 1
        self.theta_a = (self.w**2)*(r-self.theta_p) - 2*self.z*self.w*self.theta_v
     #step 2
        self.theta_v += self.theta_a*self.dt
     #step 3
        self.theta_p += self.theta_v*self.dt

        return self.theta_p ,self.theta_v
               #position      #velocity



#testing
if __name__ == "__main__":
    ideal_knee = referencem(z=1.0, w=8.0, dt=0.05)
    
    # commands a 45 degree (0.785 rad) step
    target_command = 0.785 
    
    # Run 5 steps of the loop (50 milliseconds)
    for step in range(20):
        ideal_pos, ideal_vel = ideal_knee.update(target_command)
        print(f"Time {step * 0.01:.2f}s | Ideal Pos: {ideal_pos:.4f} rad")