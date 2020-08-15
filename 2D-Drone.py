import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import math

'''

CURRENTLY ALL DOUBLE DERIVATIVE STATES ARE ZERO
Trajectory Planning Not implemented

'''
'''
This project aims to develop a 2D quadcoptor controller.
The controlled planned to control the quadcoptor model are:
1. PID
2. LQR
3. MPC (Maybe)

The 2D quadcoptor model can be modelled as:

z'' = -g + (u1/m) * cos(phi)

y'' = -(u1/m) * sin(phi)

phi'' = u2 * 1/ I_xx

The values of the drone are picked up from the Aerial Robotics Course.
'''
##--------QUADROTOR MODEL-----------###

class Quadrotor:
    
    
    def __init__(self):
        super().__init__()
        self.MASS = 0.1800
        self.GRAVITY = 9.81
        self.Ixx = 0.00025
        self.maxF = 3.5316
        self.minF = 0.0
        self.initial_state = [-3,10,np.pi/4,0,0,0]          #the initial state vector contains all the state and their dervitaive
                                                    #The self.initial_state wil be a 1*6 vector.
        self.current_state = self.initial_state
    
    def dynamics(self,state,t,u):
        z,y,phi,zdot,ydot,phidot = state    #Unpack the state vector
        u1 , u2 = u                         #Unpack the input vector 'u' into thrust and moment vectors
    
        # state_derivative = [zdot, ydot, phidot, 
        #                     -self.GRAVITY + (u1/self.MASS) * 1,
        #                     -(u1/self.MASS)*(phi), 
        #                     u2/self.Ixx]
        
        state_derivative = [zdot, ydot, phidot, 
                            -self.GRAVITY + (u1/self.MASS) * np.cos(phi),
                            -(u1/self.MASS)*np.sin(phi), 
                            u2/self.Ixx]
                                            #Writing the derivative function as per the quadc model for odeint
        return state_derivative
    
    
    
    # def simulate(self,u,t):
    #     solvedState = self.initial_state
    #     timpepoints = len(t)
    #     for i in range(timepoints):
    #         tspan = [t[i-1],t[i]]            
    #         # temp = odeint(self.dynamics,self.current_state,t,args=(u[i],))
    #         # self.current_state = temp[1,:]
    #         self.current_state = odeint(self.dynamics,self.current_state,t,args=(u[i],))[1,:]
    #         solvedState = np.append(solvedState,self.current_state, axis=0)
    #     return solvedState
 

def simulate(TUNING_MATRIX,t):
    solvedState = Drone.initial_state
    timepoints = len(t)
    Kp_z, Kd_z , Ki_z = TUNING_MATRIX[0,:]
    Kp_y, Kd_y , Ki_y = TUNING_MATRIX[1,:]
    Kp_th, Kd_th , Ki_th = TUNING_MATRIX[2,:]
    
    for i in range(1,timepoints):
            
        u1 = Drone.MASS * ( Drone.GRAVITY + 
                            Kd_z * (des_state[2] - Drone.current_state[3]) +
                            Kp_z * (des_state[0] - Drone.current_state[0])
                            )
            
        phi_c = (-1/Drone.GRAVITY) * (Kd_y * (des_state[3] - Drone.current_state[4]) +
                                      Kp_y * (des_state[1] - Drone.current_state[1]))
            
        u2 = Drone.Ixx * (Kd_th * (-1 * Drone.current_state[5]) +
                          Kp_th * (phi_c - Drone.current_state[2]) )
        
        if u1 > Drone.maxF:
            u1 = Drone.maxF
        if u1 < 0:
            u1 = 0

        
        u = [u1,u2]
                 
        # temp = odeint(self.dynamics,self.current_state,t,args=(u[i],))
        # self.current_state = temp[1,:]
        Drone.current_state = odeint(Drone.dynamics,Drone.current_state,t,args=(u,))[1,:]
        #solvedState = np.append(solvedState,Drone.current_state, axis=0)
        solvedState = np.vstack((solvedState,Drone.current_state))
        
        
        
    return solvedState

def plotResults(solvedState,t):
    plt.plot(solvedState[:,1],solvedState[:,0],'g',label = 'Plot')
    #plt.plot(t,solvedState[:,0],'g:',label = 'Plot')
    plt.plot(t,solvedState[:,2],'b',label = 'Phi')
    plt.legend()
    plt.grid(True)
    plt.show()
    
    return None
    
# Simulation Time Parameters
simulation_time = 20 # seconds
time_points = simulation_time * 100 + 1
t = np.linspace(0,simulation_time,time_points)

# des_state = np.array([
#             [np.ones_like(t)],                      # z desired
#             [np.ones_like(t)],                      # y des
#             [np.zeros_like(t)],                     #z dot des
#             [np.zeros_like(t)],                     #y dot des
#             [np.zeros_like(t)],                     #z dot dot des
#             [np.zeros_like(t)],                     #y dot dot des
#             ]).transpose()

des_state = np.array([1,2,0,0,0,0]) # z,y,zdot,ydot,zdotdot,y dotdot

Drone = Quadrotor()
TUNING_MATRIX = np.array([
    [5,4,0],
    [3,3,0],
    [70,10,0]    
])

solvedState = simulate(TUNING_MATRIX,t)

plotResults(solvedState,t)








    
    


        