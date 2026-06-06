import control 
import numpy as py


b = 0.02      # Viscous friction (Ns/rad)
j = 607 / 1e7    # Inertia in kg*m^2 (Converted from g*cm^2)
r = 105.94/ 1000   # Resistance in Ohms (Phase-to-Phase)
ke = 0.10026     # Electrical constant in V/(rad/s) (Converted from 10.5 V/krpm)
kt = 0.0105      # Torque constant in Nm/A
l = 15.48 / 1e6     # Inductance in H (Phase-to-Phase)

dt = 0.01        # Execution time step (100 Hz / 10 milliseconds)
safe_current = 10.0

v = 40.0


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


shishdsn
 momsdoflmsd[FloatingPointError
             
             dfpsdkfpfkds
             ]