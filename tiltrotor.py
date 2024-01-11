import numpy as np 
import roboticstoolbox as rtb 
from spatialmath import * 
from spatialmath.base import *
import spatialmath.base.symbolic as sym
import sympy
from sympy import Matrix, symbols, cos, sin
from solutions import generate_wrench 
import matplotlib.pyplot as plt

class TiltRotor(): 

    def __init__(self, L, km, kf): 
        self.L = L 
        self.km = km 
        self.kf = kf 
        self.a1, self.b1, self.=sympy.symbols("a1,b1,w1,a1,b2,w2,a3,b3,w3,a4,b4,w4,")
        self.prop2body = 