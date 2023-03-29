import numpy as np
import matplotlib
import matplotlib.pyplot as plt

class PID:      #PID arguments needed to be changed ASAP !!! 
        
    def __init__(self, kp, ki, kd, outMin, outMax):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.outMin = outMin
        self.outMax = outMax

        self.lastError = 0
        self.sumError = 0
        self.lastTime = 0

    def resetValues(self):
        self.lastError = 0
        self.sumError = 0

    def pidExecute(self, should, actValue):   #should value == desired value & acValue == Actual value obtained from sensor data/solving dynamics of system
        
        timeChange = 0.1
        error = should - actValue
        newErrorSum = self.sumError + (error * timeChange)

        self.sumError = newErrorSum
        dError = (error - self.lastError) / timeChange
        output = (self.kp * error) + (self.ki * self.sumError) + (self.kd * dError)
        self.lastError = error
        if(output > self.outMax):
            output = self.outMax
        if(output < self.outMin):
            output = self.outMin 
        return output

'''
For Altitude control- the roll, pitch angles and their derivatives are zero
So, the equation of dynamics along Z simplifies as:
'''
class solver_Z:
    
    def __init__(self, k1, k2, k3, k4, k5, k6, I1, I2, I3, m, l):
        
        self.dt = 0.1   # define time interval dt for the iterative solution

        self.k1 = k1
        self.k2 = k2
        self.k3 = k3
        self.k4 = k4
        self.k5 = k5
        self.k6 = k6
        
        self.m = m
        self.l = l
        self.g = 9.8    # gravitaional acceleration on Earth

        self.I1 = I1
        self.I2 = I2
        self.I3 = I3

    def F_Z(self, u1, Z):     
        return u1 - self.g - (self.k3/self.m)*Z

    #Need changes in solver -- donot iterate in in solver_z : do it expicitly again //compulsion to change
    def solve(self, u1, z_dot, z):
        '''
        tpoints = arrange(0, 10, dt)
        zpoints = []
        z_dotpoints = []
        z_dot = self.z_dot
        z = self.z
        '''

        l1 = self.dt * z_dot
        k1 = self.dt * self.F_Z(u1, z_dot)

        l2 = self.dt * (z_dot + 0.5*self.k1)
        k2 = self.dt * self.F_Z(u1, z_dot + 0.5*self.k1)

        l3 = self.dt * (z_dot + 0.5*self.k2)
        k3 = self.dt * self.F_Z(u1, z_dot + 0.5*self.k2)

        l4 = self.dt * (z_dot + self.k3)
        k4 = self.dt * self.F_Z(u1, z_dot + self.k3)

        z += (l1 + 2*l2 + 2*l3 + l4)/6
        z_dot += (k1 + 2*k2 + 2*k3 + k4)/6
        
        return z, z_dot

##############################################################################
############################### z solver class ###############################
##############################################################################
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
#Input model parameters
# g = 9.8 and dt = 0.1 are defined within the class
k1 = 0.01
k2 = 0.01
k3 = 0.01
k4 = 0.012
k5 = 0.012 
k6 = 0.012

I1 = 1.25         
I2 = 1.25
I3 = 2.5

m = 2
l = 0.2
g = 9.8

# create an object instance of the z-solver
z_solve = solver_Z(k1, k2, k3, k4, k5, k6, I1, I2, I3, m, l)

##############################################################################
############################### pid solver class #############################
##############################################################################
kp = 2
ki = 1
kd = 2
outMin = 0.5*g
outMax = 1.5*g


pid = PID(kp, ki, kd, outMin, outMax)

##############################################################################
################################## Main Code #################################
##############################################################################

#Vertical thrust input (per unit mass) to be given here. Assuming the initial thrust enough to overcome gravity
u1 = g

z_dot = 0.0 
z = 0.0
t = 0.0
dt = 0.1

zpoints = [z]
z_dotpoints = [z_dot]
t_steps = [t]
u_inputs = [u1]

'''
The desired value of Altitude to be defined as the input for PID controller;
Considering desired altitude as 10m 
'''
z_des = 10
epsilon = 0.2

error = z_des - z
error_points = [error]

# while (abs(z - z_des) > epsilon) and (t < 50):
while (t < 50):
    
    print(f"\ntime    : {t}")
    print(f"z       : {z}")
    print(f"z_dot   : {z_dot}")
    print(f"u input : {u1}")

    t += dt
    z, z_dot = z_solve.solve(u1, z_dot, z)                  #Check whether this is complete or anything is to be mentioned in brackets
    
    zpoints.append(z)
    z_dotpoints.append(z_dot)
    t_steps.append(t)
    u_inputs.append(u1)
    error = z_des - z
    error_points.append(error)
    
    u1 = pid.pidExecute(z_des ,z)

plt.plot(t_steps, u_inputs)
plt.title("Control Input vs Time")

plt.plot(t_steps, zpoints)
plt.title("Z vs Time")

plt.plot(t_steps, z_dotpoints)
plt.title("Z_dot vs Time")

plt.plot(t_steps, error_points)
plt.title("Error vs Time")