import numpy as py
import control as ct
import matplotlib.pyplot as plt

# 1. PHYSICAL CONSTANTS & CONVERSIONS

b = 0.02      # Viscous friction (Ns/rad)
j = 607 / 1e7    # Inertia in kg*m^2 (Converted from g*cm^2)
r = 12.78/ 1000   # Resistance in Ohms (Phase-to-Phase)
ke = 0.10026     # Electrical constant in V/(rad/s) (Converted from 10.5 V/krpm)
kt = 0.0105      # Torque constant in Nm/A
l = 15.3/ 1e6     # Inductance in H (Phase-to-Phase)  

dt = 0.01        # Execution time step (100 Hz / 10 milliseconds)
safe_current = 10.0

A_c = py.array([[0, 1 , 0]
                ,[0 ,-b/j, kt/j],
                [0, -ke/l , -r/l]])

B_c = py.array([[0],   #the input matirx
                [0],
                [1/l]])

# C matrix: We observe both states (Velocity and Current)
C_c = py.array([[1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0 , 0.0, 1.0]
                ])

# D matrix: No instantaneous feedforward from input to output
D_c = py.array([[0.0],
                [0.0],
                [0.0]])


sys_c = ct.ss(A_c, B_c , C_c , D_c)

print(sys_c.poles())

t, y = ct.impulse_response(sys_c)
velocity = y[0].flatten() # State 1: Angular Velocity (rad/s)
current = y[1].flatten()   # State 2: Current (Amps)


fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
#velocity

ax1.plot(t, velocity, color='blue', linewidth=2)
ax1.set_title('Impulse Response: Motor Angular Velocity')
ax1.set_ylabel('Velocity (rad/s)')
ax1.grid(True)

#current
ax2.plot(t, current, color='red', linewidth=2)
ax2.set_title('Impulse Response: Motor Phase Current')
ax2.set_xlabel('Time (seconds)')
ax2.set_ylabel('Current (A)')
ax2.grid(True)

plt.show()