import numpy as np
import math
import time
import PID_controller_new as LIMO_PID_sim

class Tracker:

    def __init__(self, **kwargs) -> None:
        """
        Tracker object used to follow a sequence of waypoints in a trajectory.
        Using this sends all the infrastructure necessary to receive pose
        information form the motion capture and send controls to the robot to track a trajectory
        """

        # Frequency at which commands are sent to limo in Hz (max is 10hz in real robot) (in proportion to scale of world)
        self.dt = kwargs.get('dt', 1e-2)
        self.transmissionRate = kwargs.get('transmissionRate', None)
        if self.transmissionRate is not None:
            self.transmissionRate = 1/self.dt

        # self.rate = rospy.Rate(self.transmissionRate)

        #pid tolerance is the convergence radius to the waypoint
        self.pid_tolerance = kwargs.get('pid_tolerance', 0.1)

        # Define agent state
        self.x = np.array([
            [0.],
            [0.],
            [0.],
            [0.]
        ])

        # Create PID controller object
        self.pid = kwargs.get('pid', LIMO_PID_sim.PID())
    
    def unicycleModelStep(self, x_t:np.ndarray,u:np.ndarray,dt:float)->np.ndarray:
        x_t[3,0]=0
        x1_dot = u[1,0]*math.cos(x_t[2,0])
        x2_dot = u[1,0]*math.sin(x_t[2,0])
        theta_dot = u[0,0]
        v = u[1,0]
        x_dot = np.array([
            [x1_dot*dt],
            [x2_dot*dt],
            [theta_dot*dt],
            [v]
            ])
        return x_t+x_dot
    
    def trackTrajectoryPID(
            self,
            trajectory: np.ndarray,
            ex=None,
            stab_time: int = 1000,  # varies
            relin_steps: int = 1,
            fig=None,
            ax=None,
    ) -> list:
        """
        This function receives a trajectory of states and, using the motion capture
        to localize, sends control commands to the limo to track the trajectory.
        Controls are found using a PID.

        :param trajectory: array of N waypoints that you want the robot to follow
            -> 4xN NumPy array
        :param stab_time: approximate number of time steps it takes to rach each point
            -> int
        :param relin_steps: number of time steps between each relinearization
            -> int
        """
        plot_x = []
        plot_y = []

        values = []
        Values_u_steer = []
        Values_u_vel = []
        Values_x = []
        Values_new_x = []
        Values_t = []
        t = 0

        # iterate through sequence of waypoints
        print("traj shape:")
        print(trajectory.shape)
        for i in range(trajectory.shape[1]):
            # isolate current waypoint
            xd = trajectory[:, i:i+1]
            # find the initial error values for the PID
            # explanations of the error found in the LIMO_PID_sim.py file
            unit_theta = np.array([
                [math.cos(self.x[2, 0])],
                [math.sin(self.x[2, 0])]
            ])
            ref = xd[:2]-self.x[:2]
            e_steer_prev = np.cross(unit_theta.T, ref.T)[0]
            e_steer_int = 0
            e_vel_prev = np.dot(unit_theta.T, ref)[0, 0]
            e_vel_int = 0
            dist = 0
            # Approach the next waypoint for stab_time time steps
            for count in range(stab_time, 1, -1):

                # Find the controls from the PID # u(t) output values, errors, integral error
                u_steer, e_steer_prev, e_steer_int = self.pid.steerControl(
                    self.x, xd, e_steer_prev, e_steer_int)
                u_vel, e_vel_prev, e_vel_int = self.pid.speedControl(
                    self.x, xd, e_vel_prev, e_vel_int)

                Values_u_steer.append(u_steer)
                Values_u_vel.append(u_vel)
                Values_x.append(self.x)
                Values_new_x.append(xd)
                Values_t.append(t)

                # Apply the controls to the nonlinear model
                # self.x = nonlinearModelStep(self.x, np.array([[u_steer], [u_vel]]) , self.dt)
                t = t+self.dt
                # print(t)

                self.x = self.unicycleModelStep(
                    self.x, np.array([[u_steer], [u_vel]]), self.dt)

                # np is numpy class, linalg is method in numpy (np) class, xd is waypoints value - self.x coordinate value = distance
                dist = np.linalg.norm(xd[0:2, 0] - self.x[0:2, 0])
                if dist < self.pid_tolerance:
                    break


        # puts each list of all values for each step into one list
        values = [Values_u_steer, Values_u_vel,
                  Values_x, Values_new_x, Values_t] 
        return values


    