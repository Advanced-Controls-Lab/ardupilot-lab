import sympy as sym 
import math
from sympy import Matrix, symbols, cos, sin

w1, w2,w3, w4, a1, a2,a3,a4, b1,b2,b3,b4 = symbols('w1 w2 w3 w4 a1 a2 a3 a4 b1 b2 b3 b4')
c = math.sqrt(2)/2
kf = 1.6e-6
km = 2.8e-8
l = 0.16
coeff_matrix = Matrix([ 
    (sin(a1) * cos(b1)*kf * (w1 **2)),(sin(a2) * cos(b2) * kf* (w2 **2)),(sin(a3) * cos(b3) * kf* (w3 **2)),(sin(a4) * cos(b4) * kf*(w4 **2)),
    (sin(b1) *kf*  (w1 **2)),(sin(b2)  * kf* (w2 **2)),(sin(b3) *kf* (w3 **2)),(sin(b4) *kf*(w4 **2)),
    (-cos(a1) * cos(b1) *kf* (w1 **2)),(-cos(a2) * cos(b2) *kf* (w2 **2)),(-cos(a3) * cos(b3) *kf* (w3 **2)),(-cos(a4) * cos(b4) *kf* (w4 **2)),
    (-c*l*cos(a1)*cos(b1)*kf + km*sin(a1)*cos(b1)),(c*l*cos(a2)*cos(b2)*kf + km*sin(a2)*cos(b2)),(c*l*cos(a3)*cos(b3)*kf - km*sin(a3)*cos(b3)),(-c*l*cos(a4)*cos(b4)*kf - km*sin(a4)*cos(b4)),
    (c*l*cos(a1)*cos(b1)*kf + km*sin(b1)),(-c*l*cos(a2)*cos(b2)*kf + km*sin(b2)),(c*l*cos(a3)*cos(b3)*kf - km*sin(b3)),(-c*l*cos(a4)*cos(b4)*kf - km*sin(b4)),
    (-c*l*sin(a1)*cos(b1)*kf + c*l*sin(b1)*kf + km*cos(a1)*cos(b1)),(c*l*sin(a2)*cos(b2)*kf- c*l*sin(b2)*kf + km*sin(a2)*cos(b2)),(c*l*cos(a3)*cos(b3)*kf +c*l*sin(b3)*kf-km*sin(a3)*cos(b3)),(-c*l*cos(a4)*cos(b4)*kf - c*l*sin(b4)*kf - km*sin(a4)*cos(b4))
])

coeff_matrix = Matrix([ 
    (sin(a1) * cos(b1)*kf * (w1 **2)) - (sin(a2) * cos(b2) * kf* (w2 **2)) + (sin(a3) * cos(b3) * kf* (w3 **2)) - (sin(a4) * cos(b4) * kf*(w4 **2)),
    -(sin(b1) *kf* (w1 **2)) + (sin(b2) * kf* (w2 **2)) - (sin(b3) *kf* (w3 **2))+(sin(b4) *kf*(w4 **2)),
    (-cos(a1) * cos(b1) *kf* (w1 **2)) - (-cos(a2) * cos(b2) *kf* (w2 **2))+(-cos(a3) * cos(b3) *kf* (w3 **2))-(-cos(a4) * cos(b4) *kf* (w4 **2)),
    (-c*l*cos(a1)*cos(b1)*kf + km*sin(a1)*cos(b1))+(c*l*cos(a2)*cos(b2)*kf + km*sin(a2)*cos(b2))+(c*l*cos(a3)*cos(b3)*kf - km*sin(a3)*cos(b3))+(-c*l*cos(a4)*cos(b4)*kf - km*sin(a4)*cos(b4)),
    (c*l*cos(a1)*cos(b1)*kf + km*sin(b1))+(-c*l*cos(a2)*cos(b2)*kf + km*sin(b2))+(c*l*cos(a3)*cos(b3)*kf - km*sin(b3))+(-c*l*cos(a4)*cos(b4)*kf - km*sin(b4)),
    (-c*l*sin(a1)*cos(b1)*kf + c*l*sin(b1)*kf + km*cos(a1)*cos(b1))+(c*l*sin(a2)*cos(b2)*kf- c*l*sin(b2)*kf + km*sin(a2)*cos(b2))+(c*l*cos(a3)*cos(b3)*kf +c*l*sin(b3)*kf-km*sin(a3)*cos(b3))+(-c*l*cos(a4)*cos(b4)*kf - c*l*sin(b4)*kf - km*sin(a4)*cos(b4))
])
y = Matrix([a1,b1,w1,a1,b2,w2,a3,b3,w3,a4,b4,w4])
jacobian = coeff_matrix.jacobian(y)
print(sym.shape(jacobian))
u_y = 