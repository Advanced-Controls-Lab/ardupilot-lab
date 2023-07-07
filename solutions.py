import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpb 

mu = -(1.6 *(10**-6))/(1.94*9.81*0.25*(100/70))
Km = (2.58 * (10**-8))/(1.94*0.5*(100/70)*9.81*0.16)
L = 0.16
c = 0.707106781
A = c * L * mu
lamb = (5 * (10**-9))

coefficient_matrix = [
       [-float(mu), 0.0,0.0, -float(mu), 0.0,0.0,-float(mu), 0.0,0.0, -float(mu), 0.0, 0.0], 
       [ 0.0, -float(mu),0.0, 0.0, -float(mu), 0.0, 0.0, -float(mu), 0.0, 0.0, -float(mu), 0.0], 
       [ 0.0, 0.0, float(mu), 0.0,0.0, float(mu), 0.0,0.0, float(mu), 0.0,0.0, float(mu)], 
       [float(Km),0.0,float(A ), float(Km),0.0, -float(A), -float(Km),0.0, -float(A), -float(Km),0.0, float(A)], 
       [0.0,-float(Km),-float(A ),0.0,-float(Km), float(A),0.0, float(Km), -float(A), 0.0,float(Km), float(A)], 
       [-float(A),-float(A), float(Km), float(A),float(A), float(Km), float(A),-float(A), -float(Km ), -float(A),float(A), -float(Km)]
    ]
C = 0.0
coefficient_matrix2 = [
       [float(mu)+ C, 0.0,0.0, float(mu)+C, 0.0,0.0,float(mu)+C, 0.0,0.0, float(mu)+C, 0.0, 0.0], 
       [ 0.0, float(mu)+C,0.0, 0.0, float(mu)+C, 0.0, 0.0, float(mu)+C, 0.0, 0.0, float(mu)+C, 0.0], 
       [ 0.0, 0.0, float(mu)+C, 0.0,0.0, float(mu)+C, 0.0,0.0, float(mu)+C, 0.0,0.0, float(mu)+C], 
       [-float(Km),0.0,float(A ), -float(Km),0.0, -float(A), -float(Km),0.0, -float(A), -float(Km),0.0, float(A)], 
       [float(Km),0.0,-float(A ),float(Km),0.0, -float(A), float(Km),0.0, float(A), float(Km),0.0, float(A)], 
       [-float(A),-float(A), -float(Km), float(A),-float(A), -float(Km), float(A),float(A), -float(Km ), -float(A),float(A), -float(Km)]
    ]

coeff_array = np.matrix(np.array(coefficient_matrix))
B_array = np.matrix(np.identity(12))
B_array[np.arange(1, 12), np.arange(0,11)] = -1 
points = 500
noise = np.random.normal(0.001,0.02,6*points).reshape((6,points))
noise[2] = np.linspace(0.01, 1.0 ,points)
#noise[3] = np.linspace(0.01, 1.0, points)
lambda_array = lamb * np.identity(12)
sol_array = np.linalg.pinv((coeff_array.T.dot(coeff_array) + (lamb**2)*B_array.T.dot(B_array))).dot(coeff_array.T) 
sol_array2 = np.linalg.pinv((coeff_array.T.dot(coeff_array) + (lamb**2)*np.matrix(np.identity(12)))).dot(coeff_array.T)
outputs = sol_array.dot(noise)
np.set_printoptions(suppress=True)
print(repr(sol_array.T.reshape((72,))))
print(np.linalg.cond(sol_array))
angular_velocity = np.sqrt(np.abs(outputs[2:outputs.shape[0]:3]))/1500

beta_angles = np.remainder(outputs[np.arange(1, 12, 3)]/outputs[np.arange(2, 12, 3)], np.pi)
alpha_angles = np.remainder(outputs[np.arange(0, 10, 3)]/outputs[np.arange(2,12, 3)], np.pi) 
beta_angles *= (180/np.pi)
alpha_angles *= (180/np.pi)

time = np.linspace(0,points,num=points)
fig,axes = plt.subplots(ncols=4, nrows=2, figsize=(5,6))
counter = 1
for row in range(2): 
    for col in range(2): 
        axes[row, col].plot(noise[2], alpha_angles[counter-1].T, color='red', label="pitch angle")
        axes[row, col].plot(noise[2], beta_angles[counter-1].T, color='blue', label="roll angle")
        axes[row, col].legend()
        axes[row, col].set_title(f"Servo Angles for Motor {counter}")
        axes[row, col].set_xlabel("points(t)")
        axes[row, col].set_ylabel("Angle (degrees)")
        counter+=1


axis = ["forward", "right", "throttle", "roll", "pitch", "yaw"]
counter = 0
for row in range(2): 
    for col in range(2,4): 
        axes[row, col].plot(time, angular_velocity[counter-1].T, color='red', label="motor rpm")
        axes[row, col].legend()
        axes[row, col].set_title(f"Motor {counter +1} rad/s vs gaussian noise")
        axes[row, col].set_xlabel(f"Data Points")
        axes[row, col].set_ylabel("Motor RPM")
        counter+=1

plt.show()