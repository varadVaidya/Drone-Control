import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import matplotlib.animation as animation
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
        self.initial_state = np.array([np.random.randint(-5,5), 
                                         np.random.randint(-5,5),   np.random.uniform(-np.pi/3,np.pi/3),0,0,0])        
        #the initial state vector contains all the state and their dervitaive
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
    # plt.plot(solvedState[:,1],solvedState[:,0],'k',label = 'Trajectory')
    # plt.plot(t,solvedState[:,0],'r:',label = 'Z-axis')
    # plt.plot(t,solvedState[:,1],'g:',label = 'Y-axis')
    # plt.plot(t,solvedState[:,2],'b',label = 'Phi')
    # plt.legend()
    # plt.grid(True)
    # plt.show()
    
    fig = plt.figure()
    spec = fig.add_gridspec(3,4)
    
    ax1 = fig.add_subplot(spec[0:3,0:3])
    ax1.set_title('Z-Y Plane Trajectory')
    ax1.grid(True)
    ax1.plot(solvedState[:,1],solvedState[:,0],'k',label = 'Trajectory')

        
    ax2 = fig.add_subplot(spec[0,3])
    ax2.set_title('Z Trajectory')
    ax2.grid(True)
    ax2.plot(t,solvedState[:,0],'r:',label = 'Z-axis')
    
    ax3 = fig.add_subplot(spec[1,3])
    ax3.set_title('Y Trajectory')
    ax3.grid(True)
    ax3.plot(t,solvedState[:,1],'g:',label = 'Y-axis')
    
    ax4 = fig.add_subplot(spec[2,3])
    ax4.set_title('Phi Trajectory')
    ax4.grid(True)
    ax4.plot(t,solvedState[:,2],'b',label = 'Phi')
    plt.legend()
    plt.show()
    
    
    return None
    


# des_state = np.array([
#             [np.ones_like(t)],                      # z desired
#             [np.ones_like(t)],                      # y des
#             [np.zeros_like(t)],                     #z dot des
#             [np.zeros_like(t)],                     #y dot des
#             [np.zeros_like(t)],                     #z dot dot des
#             [np.zeros_like(t)],                     #y dot dot des
#             ]).transpose()


def anime():
    l = (Drone.Ixx * 12/ Drone.MASS) * (1 / 2)                  # length of drone
    l1=l+0.15
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, aspect='equal')
    
    ax.grid()
    
    line, = ax.plot([], [], '-', lw=5.5)
    lined, = ax.plot([], [], '-', lw=5.5)
    line1, = ax.plot([], [], '-', lw=3)
    lined1, = ax.plot([], [], '-', lw=3)

    def init():
        line.set_data([], [])
        lined.set_data([], [])
        line1.set_data([], [])
        lined1.set_data([], [])
        
        return lined, line, lined1, lined

    def animate(i):
        global N
        global T
        while (i < len(t)):
            y = [solvedState[i, 1] - (l / 2) * np.cos(solvedState[i, 2]),
                solvedState[i, 1] + (l / 2) * np.cos(solvedState[i, 2])]
            z = [solvedState[i, 0] - (l / 2) * np.sin(solvedState[i, 2]),
                solvedState[i, 0] + (l / 2) * np.sin(solvedState[i, 2])]

            yd = [des_state[1] - (l / 2) * np.cos(des_state[2]), des_state[1] + (l / 2) * np.cos(des_state[2])]
            zd = [des_state[0] - (l / 2) * np.sin(des_state[2]), des_state[0] + (l / 2) * np.sin(des_state[2])]

            y1 = [solvedState[i, 1] - (l1 / 2) * np.cos(solvedState[i, 2]),
                solvedState[i, 1] + (l1 / 2) * np.cos(solvedState[i, 2])]
            z1 = [solvedState[i, 0] - (l1 / 2) * np.sin(solvedState[i, 2]),
                solvedState[i, 0] + (l1 / 2) * np.sin(solvedState[i, 2])]

            yd1 = [des_state[1] - (l1 / 2) * np.cos(des_state[2]), des_state[1] + (l1 / 2) * np.cos(des_state[2])]
            zd1 = [des_state[0] - (l1 / 2) * np.sin(des_state[2]), des_state[0] + (l1 / 2) * np.sin(des_state[2])]
            
            ax.set_xlim(solvedState[i,1]-1,solvedState[i,1]+1)
            ax.set_ylim(solvedState[i,0]-1,solvedState[i,0]+1)
            
                

            line.set_data(y, z)
            lined.set_data(yd, zd)
            line1.set_data(y1, z1)
            lined1.set_data(yd1, zd1)
            plt.draw()

            return lined1, line1, lined, line
    
    

    ani = animation.FuncAnimation(fig, animate,
                                  interval=6.667, blit=False, init_func=init, repeat = False,frames=1000)
    ani.save('2D.mp4',writer='ffmpeg',fps=25,bitrate=1800)

    plt.show()
    return None


# Simulation Time Parameters
simulation_time = 10 # seconds
time_points = simulation_time * 100 + 1
t = np.linspace(0,simulation_time,time_points)



Drone = Quadrotor()
des_state = np.array([np.random.randint(-5,5),np.random.randint(-5,5),0,0,0,0]) # z,y,zdot,ydot,zdotdot,y dotdot
print("Initial State:",Drone.initial_state)
print("Final State:",des_state)

TUNING_MATRIX = np.array([
    [5,4,0],
    [5,4,0],
    [70,10,0]    
])

solvedState = simulate(TUNING_MATRIX,t)
anime()
plotResults(solvedState,t)








    
    


        