
import numpy as np
from numpy import arange
from matplotlib.pyplot import plot, show

#declaring variables of quadrotor charecteristics
'''
Model parameters-
k1: drag coefficient
k2: drag coefficient
k3: drag coefficient
k4: roll drag coefficient
k5: pitch drag coefficient
k6: yaw drag coefficient

m: mass of the quadcopter

l: length of the arm

I1: Inertia along roll
I2: Inertia along pitch
I3: Inertia along yaw

Model parameters reference taken from a research paper mentioned in the documentation
'''
I1 = 1.25         
I2 = 1.25
I3 = 2.5
k1 = 0.01
k2 = 0.01
k3 = 0.01
k4 = 0.012
k5 = 0.012 
k6 = 0.012
m = 2
l = 0.2 
g = 9.8
'''
RK method of 4th order for approximating solution of system
Transforming second order equations into system of first order equations fpr all the 6 DOF.

Method opted - Solving system of 6 equations simultaneously using RK method of 4th order

Equations are solved simultaneously because each state variable depends on other variables (even with non-linear dependency)
So, Runge kutta method of 4th order is opted for better convergence of solution

'''

def F_X(u1, phi, theta, psi, X):
    return u1*(np.cos(phi)*np.sin(theta)*np.cos(psi)) - (k1/m)*X

def F_Y(u1, phi, psi, Y):
    return u1*(np.sin(phi)*np.sin(psi)*np.cos(psi)-np.cos(phi)*np.sin(psi)) - (k2/m)*Y

def F_Z(u1, phi, psi, Z):
    return u1*(np.cos(phi)*np.cos(psi)) - g - (k3/m)*Z

def F_phi(u4, cap_phi):
    return u4 - (k6/I3)*cap_phi

def F_theta(u2, cap_theta):
    return u2 - (k4*l/I1)*cap_theta

def F_psi(u3, cap_psi):
    return u3 - (k5*l/I2)*cap_psi

def F_phi(u4, cap_phi):
    return u4 - ((k6*cap_phi)/I3)

#define time interval dt = 0.2 sec
dt = 0.2
tpoints = arange(0,10,dt)
zpoints = []
z_dotpoints = []
xpoints = []
x_dotpoints = []
ypoints = []
y_dotpoints = []
thetapoints = []
theta_dotpoints = []
phipoints = []
phi_dotpoints = []
psipoints = []
psi_dotpoints = []
'''
Initialise all the values of state variables to zero

'''
z = 0.0
z_dot = 0.0
x = 0.0
x_dot = 0.0
y = 0.0
y_dot = 0.0
x = 0.0
x_dot = 0.0
phi = 0.0
phi_dot = 0.0
psi = 0.0
psi_dot = 0.0
theta = 0.0
theta_dot = 0.0
  


#define all the simplified control inputs. (Assuming arbitrary values)
u1  = g
u2 = 0.0
u3 = 0.0
u4 = 0.0

#Solver for Z coordinate considering all parameters in the dynamics equations
for t in tpoints:
    #adding the state variable values to corresponding lists (appending)
    zpoints.append(z)
    z_dotpoints.append(z_dot) 
    xpoints.append(x)
    x_dotpoints.append(x_dot) 
    ypoints.append(y)
    y_dotpoints.append(y_dot) 
    psipoints.append(psi)
    psi_dotpoints.append(psi_dot) 
    phipoints.append(phi)
    phi_dotpoints.append(phi_dot) 
    thetapoints.append(theta)
    theta_dotpoints.append(theta_dot) 

    a1 = dt*x_dot 
    b1 = dt*F_X(u1, phi, theta, psi, x_dot)
    c1 = dt*y_dot 
    d1 = dt*F_Y(u1, phi, psi, y_dot)
    e1 = dt*z_dot 
    f1 = dt*F_Z(u1, phi, psi, z_dot)
    g1 = dt*theta_dot 
    h1 = dt*F_theta(u2, theta_dot)
    i1 = dt*psi_dot 
    j1 = dt*F_psi(u2, psi_dot)
    k1 = dt*phi  
    l1 = dt*F_phi(u2, phi_dot)

    a2 = dt*(z_dot + 0.5*b1)
    b2 = dt*F_X(u1, phi+0.5*k1, theta+0.5*g1, psi+0.5*i1 ,x_dot+0.5*b1)
    c2 = dt*(z_dot + 0.5*d1)
    d2 = dt*F_Y(u1, phi+0.5*k1, psi+0.5*i1, y_dot+0.5*d1)
    e2 = dt*(z_dot + 0.5*e1)
    f2 = dt*F_Z(u1, phi+0.5*k1, theta+0.5*g1, z_dot+0.5*e1)
    g2 = dt*(theta_dot + 0.5*h1)
    h2 = dt*F_theta(u2, theta_dot+0.5*h1) 
    i2 = dt*(psi_dot + 0.5*j1)
    j2 = dt*F_psi(u2, psi_dot+0.5*j1)
    k2 = dt*(z_dot + 0.5*l1)
    l2 = dt*F_phi(u2, phi_dot+0.5*l1)

    a3 = dt*(z_dot + 0.5*b2)
    b3 = dt*F_X(u1, phi+0.5*k2, theta+0.5*g2, psi+0.5*i2, x_dot+0.5*b2)
    c3 = dt*(z_dot + 0.5*d2)
    d3 = dt*F_Y(u1, phi+0.5*k2, psi+0.5*i2, y_dot+0.5*d2)
    e3 = dt*(z_dot + 0.5*e2)
    f3 = dt*F_Z(u1, phi+0.5*k2, theta+0.5*g2, z_dot+0.5*e2)
    g3 = dt*(theta_dot + 0.5*h2)
    h3 = dt*F_theta(u2, theta_dot+0.5*h2) 
    i3 = dt*(psi_dot + 0.5*j2)
    j3 = dt*F_psi(u2, psi_dot+0.5*j2)
    k3 = dt*(phi_dot + 0.5*l2)
    l3 = dt*F_phi(u2, phi_dot+0.5*l2) 

    a4 = dt*(x_dot + b3)
    b4 = dt*F_X(u1, phi+k3, theta+g3, psi+0.5*i3 ,x_dot+b3)
    c4 = dt*(y_dot + d3)
    d4 = dt*F_Y(u1, phi+k3, psi+i3, y_dot+d3)
    e4 = dt*(z_dot + e3)
    f4 = dt*F_Z(u1, phi+k3, theta+g3, z_dot+e3)
    g4 = dt*(theta_dot + h3)
    h4 = dt*F_theta(u2, theta_dot+h3) 
    i4 = dt*(psi_dot + j3)
    j4 = dt*F_psi(u2, psi_dot+j3)
    k4 = dt*(phi_dot + l3) 
    l4 = dt*F_phi(u2, phi_dot+l3) 

    x += (a1 + 2*a2 + 2*a3 + a4)/6
    x_dot += (b1 + 2*b2 + 2*b3 + b4)/6

    y += (c1 + 2*c2 + 2*c3 + c4)/6
    y_dot += (d1 + 2*d2 + 2*d3 + d4)/6

    z += (e1 + 2*e2 + 2*e3 + e4)/6
    z_dot += (f1 + 2*f2 + 2*f3 + f4)/6

    theta += (g1 + 2*g2 + 2*g3 + g4)/6
    theta_dot += (h1 + 2*h2 + 2*h3 + h4)/6

    psi += (i1 + 2*i2 + 2*i3 + i4)/6
    psi_dot += (j1 + 2*j2 + 2*j3 + j4)/6

    phi += (k1 + 2*k2 + 2*k3 + k4)/6
    phi_dot += (l1 + 2*l2 + 2*l3 + l4)/6

    u1 += dt*4     #update the value of u1(Thrust input/control signal). Input / users choice
    u2 += dt*4
    u3 += dt*4
    u4 += dt*4
'''
Considered that the control variables i.e., u1, u2, u3, u4 are continously increasing.
This can also be solved by taking control inputs and iterating it explicitly based on the input and time step.

'''

