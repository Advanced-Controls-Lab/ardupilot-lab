import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

mu = -(1.6*(10**-6))/(1.94*9.81*0.25*(100/70))
Km = (2.58 *(10**-8))/(1.94*0.5*(100/70)*9.81*0.16)
L = 0.16
c = 0.707106781
A = c * L * mu

coefficient_matrix = [
       [-float(mu), 0.0,0.0, -float(mu), 0.0,0.0,-float(mu), 0.0,0.0, -float(mu), 0.0, 0.0], 
       [ 0.0, -float(mu),0.0, 0.0, -float(mu), 0.0, 0.0, -float(mu), 0.0, 0.0, -float(mu), 0.0], 
       [ 0.0, 0.0, float(mu), 0.0,0.0, float(mu), 0.0,0.0, float(mu), 0.0,0.0, float(mu)], 
       [float(Km),0.0,float(A), float(Km),0.0, -float(A), -float(Km),0.0, -float(A), -float(Km),0.0, float(A)], 
       [0.0,-float(Km),-float(A),0.0,-float(Km), float(A),0.0, float(Km), -float(A), 0.0,float(Km), float(A)], 
       [-float(A),-float(A), float(Km), float(A),float(A), float(Km), -float(A),float(A), -float(Km ), float(A),-float(A), -float(Km)]
    ]

coeff_array = np.array(coefficient_matrix)
points = 500
noise = np.random.normal(0.01,0.02,6*points).reshape((6,points))
noise[1] = np.linspace(0.01, 1.0 ,points)
noise[3] = np.linspace(0.01, 1.0, points)
outputs = np.linalg.pinv(coeff_array).dot(noise)
np.set_printoptions(suppress=True)
#print(repr(np.linalg.pinv(coeff_array).T.reshape((72,))))
print(np.linalg.cond(np.linalg.pinv(coeff_array)))
angular_velocity = np.sqrt(np.abs(outputs[2:outputs.shape[0]:3]))/1500

alpha_angles = np.zeros(outputs.shape)
beta_angles = np.zeros(outputs.shape)
lowpass_filter = 0
previous_beta = [0,0,0,0] 
previous_alpha = [0,0,0,0] 
for j in range(0, outputs.shape[1]):
    counter = 0
    for i in range(0,outputs.shape[0],3):

        if previous_alpha[counter] == 0 or previous_beta[counter] == 0 or lowpass_filter == 0:
            alpha_angles2 = (outputs[i,j]/outputs[i+2,j]) 
            alpha_angles[i,j] = np.remainder(alpha_angles2, np.pi) * (180/np.pi)
            beta_angles2 = (outputs[i+1,j]/outputs[i+2,j]) 
            beta_angles[i,j] = np.remainder(beta_angles2, np.pi) * (180/np.pi)
            previous_alpha[counter] = alpha_angles[i//3,j]
            previous_beta[counter] = beta_angles[i//3,j]
            counter+=1
        else: 
            alpha_angles2 = (outputs[i,j]/outputs[i+2,j]) 
            alpha_angles[i//3,j] = 0.5 * (np.remainder(alpha_angles2, np.pi) * (180/np.pi) - previous_alpha[counter]) +(previous_alpha[counter])
            beta_angles2 = (outputs[i+1,j]/outputs[i+2,j]) 
            beta_angles[i//3,j] = 0.5 *(np.remainder(beta_angles2, np.pi) * (180/np.pi) - previous_beta[counter]) + (previous_beta[counter])
            previous_alpha[counter] = alpha_angles[i//3,j]
            previous_beta[counter] = beta_angles[i//3,j] 
            counter+=1   

time = np.linspace(0,points,num=points)
fig,axes = plt.subplots(ncols=3, nrows=2, figsize=(5,6))
counter = 1
for row in range(2): 
    for col in range(2): 
        axes[row, col].plot(noise[3], alpha_angles[counter-1][:], color='red', label="pitch angle")
        axes[row, col].plot(noise[3], beta_angles[counter-1][:], color='blue', label="roll angle")
        axes[row, col].legend()
        axes[row, col].set_title(f"Servo Angles for Motor {counter}")
        axes[row, col].set_xlabel("points(t)")
        axes[row, col].set_ylabel("Angle (degrees)")
        counter+=1

'''
axis = ["forward", "right", "throttle", "roll", "pitch", "yaw"]
counter =  0
for row in range(2): 
    for col in range(2,4): 
        axes[row, col].plot(time, angular_velocity[counter-1][:], color='red', label="motor rpm")
        axes[row, col].legend()
        axes[row, col].set_title(f"Motor {counter +1} rad/s vs gaussian noise")
        axes[row, col].set_xlabel(f"Data Points")
        axes[row, col].set_ylabel("Motor RPM")
        counter+=1
'''
plot_colors = ['turquoise', 'steelblue', 'orchid', 'red']
pitch_angles = ["alpha 1", "alpha 2 ", "alpha 3", "alpha 4"]
roll_angles = ["beta 1", "beta 2 ", "beta 3", "beta 4"]
print(beta_angles)
for i in range(0, outputs.shape[0], 3):
    print(i) 
    axes[0, 2].plot(noise[3], beta_angles[(i)][:], color=plot_colors[(i-1)//3], label=roll_angles[(i-1)//3])
    axes[1, 2].plot(noise[3], alpha_angles[(i)][:], color=plot_colors[(i-1)//3], label=pitch_angles[(i-1)//3])
    axes[0, 2].set_title("Roll Angles plotted against each other")
    axes[1, 2].set_title("Pitch Angles plotted against each other")
    axes[0, 2].legend()
    axes[1,2].legend()
plt.show()

