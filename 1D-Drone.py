# Import all the necessary lib

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

'''
This project aims to develop a 1-D drone control which is controlled by PID and may be extended to other forms of controllers.

The controller will control the z direction of the drone.

The dymanic model for the same can be written as:

m * z_dbldot = - m * g + u ;

where u is the thurst input controlled by the controller

'''

# define the constants 
m = 0.5  # kg
g = 9.81 # m/s2
simulation_time = 5
time_points = simulation_time * 100 + 1
#define the function model

def zsolver(x,t,u):
    z , zdot = x
    xdot = [ zdot, -g + (u / m)]
    return xdot

# X is the vector required to solve

# Initial Condition 
x0 = [0,0]

# set the timepoints
t = np.linspace(0,simulation_time,time_points)

# give the step input
u = np.zeros(time_points)
u[101: ] = m * g

# Arrays to store the solutions
z = np.empty_like(t)
zdot = np.empty_like(t)

# Record the intial condition in the new solution arrays
z[0],zdot[0] = x0

#solve the ODE

for i in range(1,time_points):
    tspan = [t[i-1],t[i]]

    x = odeint(zsolver,x0,t,args=(u[i],))

    # Store the solution for plotting

    z[i] , zdot[i]  = x[1]
    # giving the next initial condition to the solver
    x0 = x[1]

# Hopefully this will solve the problem

# Plot the result

plt.plot(t,u,'g:',label = 'u(t)')
plt.plot(t,z,'r-',label = 'z(t)')
plt.plot(t,zdot,'b--', label = 'zdot(t)')
plt.xlabel('Time')
plt.legend(loc = 'best')
plt.show()



 



