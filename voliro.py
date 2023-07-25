import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpb 
from sklearn.preprocessing import normalize

mu = -(1.6 *(10**-6))/(1.94*9.81*0.25*(100/70))
km = (2.58 * (10**-8))/(1.94*0.5*(100/70)*9.81*0.16)
mu = 1.6/(1.94*9.81*0.25*(100/70)) 
km = 0.0258/(1.94*0.5*(100/70)*9.81*0.16)
l = 0.16
c = np.sqrt(3)/2
A = c * l * mu


coeff_matrix = [
    [-mu,0, mu,0,0.5 *mu,0,  -0.5 * mu,0, -0.5 * mu,0, 0.5 * mu,0],
    [0,0, 0,0, c * mu,0, -c * mu,0, c * mu,0, -c * mu,0 ], 
    [0, -mu, 0, -mu, 0, -mu, 0, -mu, 0 , -mu, 0, -mu], 
    [-km,-mu* l,-km, mu * l, 0.5*km, 0.5*mu*l, 0.5*km, -0.5*mu*l,0.5*km, -0.5*mu*l, 0.5*km,0.5*mu*l], 
    [0, 0, 0, 0, c*km, c*mu*l, c*km, -mu*c*l, -km*c,mu*c*l, -km * c, -mu*c*l ],
    [-km, mu*l, km, mu*l, -0.5*km, mu*l*0.5, 0.5*km, mu*0.5*l,0.5*km, 0.5*mu*l, -0.5 * km, 0.5*mu*l]
]
'''
coeff_matrix =  [ 
    [-mu, 0, -mu, 0, -mu, 0, -mu, 0],
    [0, -mu, 0, -mu, 0, -mu, 0, -mu],
    [km,A, km, -A, -km, A, -km, -A],
    [0, -A, 0, A, 0, A, 0, -A],
    [-A, km, A, km, -A, -km, A,-km]
]
'''
coeff = normalize(np.array(coeff_matrix))
#coeff = np.array(coeff_matrix)
print(np.linalg.cond(coeff))
points = 1500
noise= np.random.normal(0.001,0.022,6*points).reshape((6,points))
#noise[0] = np.linspace(0, 1.0, points)
#noise[1] = np.linspace(0, 1.0, points)
#noise[2] = np.linspace(0, 1.0, points)

outputs = np.linalg.pinv(coeff).dot(noise)
alphas = np.arctan(outputs[0:11:2],outputs[1:12:2]) + (0.5 * np.pi)
alphas *= (180/np.pi)
angular_vel = np.sqrt(outputs[np.arange(0,11,2)] ** 2 + outputs[np.arange(1, 12, 2)]**2)

time = np.linspace(1, points, points)

fig,axes = plt.subplots(ncols=6, nrows=2, figsize=(5,6))
counter = 1
for row in range(2): 
    for col in range(3): 
        axes[row, col].plot(time, alphas[counter-1], color='red', label="pitch angle")
        axes[row, col].legend()
        axes[row, col].set_title(f"Servo Angles for Motor {counter}")
        axes[row, col].set_xlabel("points(t)")
        axes[row, col].set_ylabel("Angle (degrees)")
        counter+=1
axis = ["forward", "right", "throttle", "roll", "pitch", "yaw"]
counter = 0

for row in range(2): 
    for col in range(3,6): 
        axes[row, col].plot(time, angular_vel[counter-1], color='red', label="motor rpm")
        axes[row, col].legend()
        axes[row, col].set_title(f"Motor {counter +1} rad/s vs gaussian noise")
        axes[row, col].set_xlabel(f"Data Points")
        axes[row, col].set_ylabel("Motor RPM")
        counter+=1

plt.show()