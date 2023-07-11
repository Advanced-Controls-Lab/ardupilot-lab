import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpb 

mu = -(1.6 *(10^-6))/(1.94*9.81*0.25*(100/70));
Km = (2.58 * (10^-8))/(1.94*0.5*(100/70)*9.81*0.16);
L = 0.16;
c = 0.707106781;
A = c * L * mu;
lamb = (5 * (10**-11))

coefficient_matrix = [
      [-mu, 0.0,0.0, - mu, 0.0,0.0,- mu, 0.0,0.0, - mu, 0.0, 0.0], 
        [0.0, -mu,0.0, 0.0, - mu, 0.0, 0.0, - mu, 0.0, 0.0, - mu, 0.0], 
        [0.0, 0.0,  mu, 0.0,0.0,  mu, 0.0,0.0,  mu, 0.0,0.0,  mu], 
        [Km,0.0, A ,  Km,0.0, -A, -Km,0.0, -A, -Km,0.0,  A], 
        [0.0,-Km,-A ,0.0,-Km,  A,0.0,  Km, -A, 0.0, Km,  A], 
        [-A,-A,  Km,  A, A,  Km,  A,-A, -Km , -A, A, -Km]
    ]

coeff_array = (np.array(coefficient_matrix))
points = 500
noise = np.random.normal(0.001,0.02,6*points).reshape((6,points))
noise[2] = 0
ambient = np.array([0, 0 , 1.94*9.81, 0 ,0 , 0]).reshape((6,1))
noise += ambient
lambda_array = lamb * np.identity(12)
sol_array2 = np.linalg.pinv((coeff_array.T.dot(coeff_array) + (lamb**2)*np.matrix(np.identity(12)))).dot(coeff_array.T)
outputs = sol_array2.dot(noise)
#print(outputs.shape)
np.set_printoptions(suppress=True)
print(f" This is the solution array {repr(sol_array2.T.reshape((72,)))}")
#print(f" This is the array to subtract off for accurate results{sol_array2.dot(ambient)}")
angular_velocity = np.sqrt(np.abs(outputs[2:outputs.shape[0]:3]))
alpha_angles = np.zeros((4, points))
beta_angles = np.zeros((4, points))
alpha_angles[0] = outputs[0]/outputs[2]
alpha_angles[1] = outputs[3]/outputs[5]
alpha_angles[2] = outputs[6]/outputs[8]
alpha_angles[3] = outputs[9]/outputs[11]

beta_angles[0] = outputs[1]/outputs[2]
beta_angles[1] = outputs[4]/outputs[5]
beta_angles[2] = outputs[7]/outputs[8]
beta_angles[3] = outputs[10]/outputs[11]


beta_angles = np.remainder(beta_angles, np.pi) * (180/np.pi)
alpha_angles = np.remainder(alpha_angles, np.pi) * (180/np.pi)

time = np.linspace(0,points,num=points).reshape((500,1))
fig,axes = plt.subplots(ncols=4, nrows=2, figsize=(5,6))
counter = 1
for row in range(2): 
    for col in range(2): 
        axes[row, col].plot(time, alpha_angles[counter-1].T, color='red', label="pitch angle")
        axes[row, col].plot(time, beta_angles[counter-1].T, color='blue', label="roll angle")
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