import numpy as py
import control as ct
import can
import time

#motor properties

b = 0.3 #viscous friction (not known)
#j = 607 #inertia in g(cm^2)
j = 607/10000000 #in kgm2

r  = 170/1000  #resistance in ohms (phase to phasw)
ke = 0.10026   #electrical constant in V/(rad/s)
kt =  0.0105   #torque constant in Nm/A
l = 57/1000000 #inductance in H (phase to phase)


#i_peak = 20 
safe_current = 10 
#t_peak = 6*0.3



## these are input data of motor
theta1 =  input #velocity   
theta2  = input #acceleration
i = input # current


dt = 0.01


#A B matrices

A = py.array([[-b/j  , kt/j ],
              [-ke/l ,-r/l ]])

B = py.array([[0],
              [1/l]])

x_state = py.array([[theta1],
                    [i]])

#position bounds
pmin = 0 
pmax = 12.5



#for data from ak80-6
def feedback(data):
    raw_current = (data[3]<<4)|(data[4]>>4)
    #current = (raw_current/4095.0)*range of current
    #also for raw torque
    current = raw_current

    raw_position = (data[1]<<8)|(data[2]) 
    position = (pmax-pmin)*(raw_position)/65535 + pmin


    return current, position


def bus_connection():
    try: 
        bus = can.interface.Bus(bustype= 'slcan', channel= 'COM5', bitrate = 1000000)
        print("connection succesful")

        return bus
    
    except Exception as e :
        print(f"no connection:{e}")
        
        return None
    
bus = bus_connection()
    
# Only run the loop if the bus initialized successfully
if bus is not None:
    try:
        while True:
            msg = bus.recv(timeout = 1.0)
        
            if msg is not None: 
                current , position = feedback(msg.data)

                print(f"position is:{position}")
                
                if current > safe_current:
                    print("current limit exceeded")
                    break

    except KeyboardInterrupt:
        print('manually stopped')

    finally:
        bus.shutdown()
else:
    print("stopped, check for errors")



#now for pid controller 

