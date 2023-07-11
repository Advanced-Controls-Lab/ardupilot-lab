import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpb 
from sklearn.preprocessing import normalize

mu = -(1.6 *(10**-6))
Km = (2.58 * (10**-8))
L = 0.16
c = 0.707106781
A = c * L * mu
lamb = (7 * (10**-10))

coefficient_matrix = [
       [-float(mu), 0.0,0.0, -float(mu), 0.0,0.0,-float(mu), 0.0,0.0, -float(mu), 0.0, 0.0], 
       [ 0.0, -float(mu),0.0, 0.0, -float(mu), 0.0, 0.0, -float(mu), 0.0, 0.0, -float(mu), 0.0], 
       [ 0.0, 0.0, float(mu), 0.0,0.0, float(mu), 0.0,0.0, float(mu), 0.0,0.0, float(mu)], 
       [float(Km),0.0,float(A ), float(Km),0.0, -float(A), -float(Km),0.0, -float(A), -float(Km),0.0, float(A)], 
       [0, -Km , -A, 0, -Km , A, 0, Km, -A, 0, Km, A],
       [-A, -A,Km, A, A, Km, A, -A, -Km, -A, A, Km ]
    ]
C = 0.0

coeff_array = normalize(np.array(coefficient_matrix))
#coeff_array[0:3] /= (1.94*9.81*0.25*(100/70))
#coeff_array[3:6] /= (1.94*0.5*(100/70)*9.81*0.16)
w = np.matrix(np.identity(12))
for i in range(0,12,3): 
    w[i,i] = 0.125
    w[i+1, i+1] = 0.125 
    w[i+2, i+2] = 0.75
points = 1000
noise = np.random.normal(0.15,0.01,6*points).reshape((6,points))
noise[5] = 0.01
noise[2] = np.linspace(0, 1, points)
lambda_array = lamb * np.identity(12)
w_1 = np.linalg.inv(w)
outputs = ((w_1.dot(coeff_array.T)).dot(np.linalg.inv(coeff_array.dot(w_1.dot(coeff_array.T))))).dot(noise)
#outputs = np.linalg.pinv(coeff_array).dot(noise)
np.set_printoptions(suppress=True)
#print(repr(sol_array.T.reshape((72,))))
#print(np.linalg.cond(sol_array))
angular_velocity = np.sqrt(np.abs(outputs[2:outputs.shape[0]:3] - (1725**2)))

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
        axes[row, col].plot(noise[2], angular_velocity[counter-1].T, color='red', label="motor rpm")
        axes[row, col].legend()
        axes[row, col].set_title(f"Motor {counter +1} rad/s vs gaussian noise")
        axes[row, col].set_xlabel(f"Data Points")
        axes[row, col].set_ylabel("Motor RPM")
        counter+=1

plt.show()