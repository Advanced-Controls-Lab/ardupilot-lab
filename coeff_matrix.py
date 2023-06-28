import numpy as np 

MU = (1.6*(10**0))
Km = (2.58 *(10**-2))
TorqueLength = 0.16
angle_const = 0.707106781

coefficient_matrix = [
       [float(MU), 0.0,0.0, -float(MU), 0.0,0.0,float(MU), 0.0,0.0, -float(MU), 0.0, 0.0], 
       [ 0.0, -float(MU),0.0, 0.0, float(MU), 0.0, 0.0, -float(MU), 0.0, 0.0, float(MU), 0.0], 
       [ 0.0, 0.0, float(MU), 0.0,0.0, -float(MU), 0.0,0.0, float(MU), 0.0,0.0, -float(MU)], 
       [-float(Km),0.0,float(angle_const * MU * TorqueLength ), -float(Km),0.0, -float(angle_const* TorqueLength * MU), -float(Km),0.0, -float(angle_const*TorqueLength*MU), -float(Km),0.0, float(angle_const* TorqueLength * MU)], 
       [float(Km),0.0,-float(angle_const * MU * TorqueLength ),float(Km),0.0, -float(angle_const* TorqueLength * MU), float(Km),0.0, float(angle_const*TorqueLength*MU), float(Km),0.0, float(angle_const * TorqueLength * MU)], 
       [-float(angle_const * TorqueLength * MU),-float(angle_const * TorqueLength * MU), -float(Km), float(angle_const * TorqueLength * MU),-float(angle_const * TorqueLength * MU), -float(Km), float(angle_const * TorqueLength * MU),float(angle_const * TorqueLength * MU), -float(Km ), -float(angle_const * TorqueLength * MU),float(angle_const * TorqueLength * MU), -float(Km)]
    ]
coeff_array = np.array(coefficient_matrix)
input_array = np.array([1.0, 1.0,1.0,1.0, 1.0,1.0 ])
outputs = np.linalg.pinv(coeff_array).dot(input_array)
np.set_printoptions(suppress=True)
print(np.linalg.pinv(coeff_array).T)

for i in range(2,outputs.shape[0],3): 
    print(f"This is the angular velocity of motor { (i+1)//3} the motors {np.sqrt(np.abs(outputs[i]))}")
    print(f"This is the pitch servo {(i+1)//3} angle {outputs[i-2]/outputs[i]}")
    print(f"This is the roll servo {(i+1)//3} angle {outputs[i-1]/outputs[i]}")

