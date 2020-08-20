# Planar Quadcopter Control
## Aim
Aim of this project was to develop an 1-D and 2-D quadcopter model and use the basic control law such as PID to control the quadcopter to reach a certian desired setpoint and/or follow a planned trajectory.

![2D quadcopter simulation](gifs/2D_No_Traj.gif?raw=True "QuadCopter Simulation")
## Results
### 1D Quadcopter
The quadcopter was constrianed to move only along the z-axis of the world frame. The quadcopter had to reach a certain desired setpoint which changed w.r.t time.
![1D Quadcopter Control Results](images/1D-Drone.png?raw=True "1-D Quadcopter")


### 2D Quadcopter
The quadcopter was restriced to z-y plane. Such version of the quadcopter is called as a Planar Quadrotor. The dynamics of the system were found out by simple Newtonian-Mechanics and simulated using the odeint function from the scipy package.
To further test the controller the initial roll angle is allowed to vary randomly between -2pi/3 to 2pi/3 with each run of the code.

#### Trajectory Planning
For the planar Quadrotor to reach the desired postion in a certain manner a trajectory planner was implemented. The following planners were implemented.
* Minimum Velocity Trajectory
* Minimum Acceleration Trajectory
* Minimum Jerk Trajectory

## Results
The simulation results are as follows:
* For Minimum Velocity Trajectory:
    * Graph
    ![Graph for min. vel traj](images/2D-Min_Vel_Traj.png?raw=True "Min. Vel Traj")
    * GIF
    ![GIF for min. vel traj](gifs/2D_Min_Vel.gif?raw=True "Min. Vel Traj")

* For Minimum Acceleration Trajectory:
    * Graph
    ![Graph for min. accln traj](images/2D-Min_Accel_Traj.png?raw=True "Min. Accel Traj")
    * GIF
    ![GIF for min. vel traj](gifs/2D-Min_Accel.gif?raw=True "Min. Accel Traj")

* For Minimum Jerk Trajectory:
    * Graph
    ![Graph for min. jerk traj](images/2D-Min_Jerk_Traj.png?raw=True "Min. Jerk Traj")
    * GIF
    ![GIF for min. jerk traj](gifs/2D_Min_Jerk.gif?raw=True "Min. Jerk Traj")

## Future Work
Future work includes extending this project to 3 dimensions and using other control approaches such as Linear Quadratic Regulator (LQR) and Model Predictive Control(MPC). 



