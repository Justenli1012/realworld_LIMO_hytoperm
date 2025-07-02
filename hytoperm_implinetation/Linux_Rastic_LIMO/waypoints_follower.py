#!/usr/bin/env python3
import time
import numpy as np
import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
import threading
import matplotlib.pyplot as plt
import sys
from geometry_msgs.msg import PoseStamped, Pose  # Add Pose here
from nav_msgs.msg import Odometry
from pathlib import Path
import json

from cav_for_line_follower_new import CAV
from PID_controller_new import PID

# Global variable to store the current yaw (heading) of the robot
current_yaw = 0.0

def get_yaw_from_quaternion(orientation_q):
    global offset_angle_degrees
    """Convert quaternion to heading angle around Y-axis in radians (-π to π range)."""
    # Extract heading angle around Y-axis directly from quaternion
    # For rotation around Y-axis: heading = atan2(2*(qw*qy + qx*qz), 1 - 2*(qy*qy + qz*qz))
    qx, qy, qz, qw = orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
    
    # Heading around Y-axis (vertical)
    heading = math.atan2(2 * (qw * qy + qx * qz), 1 - 2 * (qy * qy + qz * qz))
    return heading+(math.radians(offset_angle_degrees))  #used to adjust robot heading from mocap misalignment, gives current_yaw value

    #for local limo orientation
# def orientation_callback(msg):
#     """ROS subscriber callback to get robot's local orientation."""
#     global current_yaw
#     # Extract quaternion from the pose message
#     orientation_q = msg.orientation
#     robot_yaw = get_yaw_from_quaternion(orientation_q)
#     print(f"Robot orientation (yaw): {math.degrees(robot_yaw):.2f}°")

    #for finding orientation in global world from Mocap system
def pose_callback(msg):
    """ROS subscriber callback to update current_yaw from PoseStamped message."""
    global current_yaw
    current_yaw = get_yaw_from_quaternion(msg.pose.orientation)

def is_near_target_circle(pos, target, radius=100, tolerance=0.1):
    '''
    generates a circle around each waypoint of 100mm. Used for sensing if robot is near target in a circle radius. 
    Tolerance corrects for values outside of circle to be defined as "inside the circle". 150mm used to align to the center of rigid-body of LIMO.
    returns true/false if point is within circle
    '''
    #pos: tuple (x,y), target: array[x,y,heading_vel,angular_vel]
    pos_x, pos_z = pos
    target_x, target_z = target[0, 0], target[1, 0]

    distance = math.sqrt((pos_x - target_x) ** 2 + (pos_z - target_z) ** 2)
    return (distance - radius) <= tolerance #boolean

#Used to retrieve points from Jonas Code Trajectory Files
def loadPoints(num):
    # Retrieves the points from the JSON files they are stored in
    # The num input is the trial number. The trajectory will be stored in the directory
    # with this trial number.

    # Get the current file's directory
    parent_hytoperm_dir = Path(__file__).parent.parent.parent

    # We start with loading the trajectory from individual segments
    points_dict = {}
    uts_dict = {}
    hds_dict = {}
    vels_dict = {}

    # Iterate through the number number of segments
    count = 0
    while True:
        try:
            # save the trajectory points, assuming trial folders are located in parent_hytoperm_directory
            TEST = str(parent_hytoperm_dir / f'trial{num}' / f'cycleInfo{num}_{count}_points.json')
            f = open(str(parent_hytoperm_dir / f'trial{num}' / f'cycleInfo{num}_{count}_points.json'), 'r')
            points_dict[count] = json.loads(f.readline())

            # save the trajectory controls
            f = open(str(parent_hytoperm_dir / f'trial{num}' / f'cycleInfo{num}_{count}_cntrls.json'), 'r')
            uts_dict[count] = json.loads(f.readline())

            # Get the hybrid dynamics of the region
            f = open(str(parent_hytoperm_dir / f'trial{num}' / f'cycleInfo{num}_{count}_dynams.json'), 'r')
            hds_dict[count] = np.array(json.loads(f.readline()))

            # reshape the hybrid dynamics to a 2D vector
            hds_dict[count].reshape((2, 1))
            # # Use the controls and dynamics to get the desired velocity (desired_vel = controls + hybrid_dynamics)
            # vels_dict[count] = getVels(
            #     np.array(uts_dict[count]), hds_dict[count])

            count += 1

        except:
            break

    tot = count

    # compose points and velocities into a single array
    pts = np.array(points_dict[0])
    #vels = np.array(vels_dict[0])
    for count in range(tot-1):
        pts = np.hstack((pts, np.array(points_dict[count+1])))
        #vels = np.hstack((vels, np.array(vels_dict[count+1])))

    return pts, None

#Uses loadPoints() to get waypoints into array
def getJonasWaypoints(trialnum):
    points, _ = loadPoints(trialnum) #points returns a 2xN array of np.float(values). First row is x coords, second row is y coords.

    #returns a Nx4 nparray of np.float values. [ [point1_x, point1_y, 0, 0],
    #                                            [point2_x, point2_y, 0, 0],
    #                                            ... and so on for N points ]
    points_transpose = points.T

    waypoints_all = np.hstack([points.T, np.zeros((points[0].size, 2))])   
    return waypoints_all
    

def plotLimoPosition(waypoints):
    plt.close('all')  # Closes all existing figure windows

    plt.ion()
    fig, ax = plt.subplots(figsize=(12,8))
    x_data, z_data = [], []
    line, = ax.plot([], [], 'b-', marker='o', label='Limo Path')
    
    # Plot waypoints in orange
    waypoint_x = waypoints[:, 0]  # x coordinates of waypoints
    waypoint_z = waypoints[:, 1]  # z coordinates of waypoints
    ax.plot(waypoint_x, waypoint_z, 'o', color='orange', markersize=12, 
            label='Waypoints', markeredgecolor='darkorange', markeredgewidth=2)
    
    # Add waypoint numbers
    for i, (x, z) in enumerate(zip(waypoint_x, waypoint_z)):
        ax.annotate(f'{i+1}', (x, z), xytext=(10, 10), textcoords='offset points',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='orange', alpha=0.7),
                    fontsize=10, fontweight='bold', color='white')

    # Set fixed axis limits as requested
    ax.set_xlim(3000, -5000)  # Flipped x-axis
    ax.set_ylim(-2000, 3000)

    ax.set_xlabel("X position (mm)")
    ax.set_ylabel("Z position (mm)")
    ax.set_title("Live Limo Position")
    ax.grid(True)
    ax.legend()
    plt.tight_layout()
    plt.show()

    # Store arrow reference in a list so it can be modified
    arrow_ref = [None]

    def update(x, z, heading_dir):
        x_head = heading_dir[0,0]
        y_head = heading_dir[1,0]

        x_data.append(x)
        z_data.append(z)
        line.set_xdata(x_data)
        line.set_ydata(z_data)
        
        # Remove old arrow if it exists
        if arrow_ref[0] is not None:
            arrow_ref[0].remove()
        
        # Create new arrow
        arrow_length = 700  # mm, adjust as needed
        dx = arrow_length * x_head
        dy = arrow_length * y_head
        
        arrow_ref[0] = ax.quiver(x, z, dx, dy, color='red', scale=1, scale_units='xy', 
                                angles='xy', width=0.005, headwidth=3, headlength=5)
        
        fig.canvas.draw()
        plt.pause(0.01)

    return update


def plotControlSignals():
    fig2, ax = plt.subplots(figsize=(10, 6))
    
    time_data = []
    velocity_data = []
    u_steer_data = []
    e_steer_data = []
    
    vel_line, = ax.plot([], [], 'g-', linewidth=2, label='Velocity (m/s)')
    u_steer_line, = ax.plot([], [], 'b-', linewidth=2, label='U_Steering (rad)')
    e_steer_line, = ax.plot([], [], 'r-', linewidth=2, label='Error_Steering')

    
    ax.set_xlabel("Time (seconds)")
    ax.set_ylabel("Control Values")
    ax.set_title("Real-time Velocity and Steering Commands")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_ylim(-1, 1)  # Initial range, will auto-scale
    
    plt.tight_layout()
    plt.show(block=False)
    
    start_time = time.time()
    
    def update_controls(velocity, u_steer, e_steer):
        current_time = time.time() - start_time
        time_data.append(current_time)
        velocity_data.append(velocity)
        u_steer_data.append(u_steer)
        e_steer_data.append(e_steer)
        
        vel_line.set_xdata(time_data)
        vel_line.set_ydata(velocity_data)
        u_steer_line.set_xdata(time_data)
        u_steer_line.set_ydata(u_steer_data)
        e_steer_line.set_xdata(time_data)
        e_steer_line.set_ydata(e_steer_data)
        
        if time_data:
            ax.set_xlim(max(0, time_data[-1] - 10), time_data[-1] + 1)
            ax.relim()
            ax.autoscale_view(scalex=False)
        
        fig2.canvas.draw()
        plt.pause(0.01)
    
    return update_controls



offset_angle_degrees = -90+11.5
def main():
    rospy.init_node("limo_waypoint_follower")
    rospy.Subscriber("/vrpn_client_node/limo780/pose", PoseStamped, pose_callback)
    #subscriber to get local limo heading orientation
    #rospy.Subscriber('/odom', Odometry, orientation_callback)  # Required - creates the subscription

    # Initialize limo vehicle and PID controller
    limo = CAV("limo780")

    pid = PID(
        # #For U_VEL_MAX = 0.7
        # steer_kp=0.7, steer_ki=0.13, steer_kd=2.03,
        # vel_kp=0.6, vel_ki=0.7, vel_kd=0.69,
        # dt=1/30

        # #For Straight Line
        # #For U_VEL_MAX = 0.7
        # steer_kp=0.8, steer_ki=0.25, steer_kd=1.6,
        # vel_kp=0.6, vel_ki=0.7, vel_kd=0.3,
        # dt=1/30

        # #4-Wheel Mode (Orange-light) w/ Gaussian error for velocity slow-down
        # steer_kp=0.67, steer_ki=0, steer_kd=0,
        # vel_kp=0.001, vel_ki=0, vel_kd=0,
        # dt=1/30

        # #4-Wheel Mode (Orange-light) w/o Gaussian error for velocity slow-down (No smoothing for speed+steer) FOR LINE
        # steer_kp=0.3, steer_ki=0, steer_kd=0,
        # vel_kp=0.001, vel_ki=0, vel_kd=0,
        # dt=1/30

        #4-Wheel Mode (Orange-light) w/o Gaussian error for velocity slow-down (No smoothing for speed+steer) 
        steer_kp=0.4, steer_ki=0, steer_kd=0,
        vel_kp=0.001, vel_ki=0, vel_kd=0,
        dt=1/30
    )

    dt = 1/30  # Control loop frequency = 30Hz
    distance_threshold = 100 # mm, threshold to consider waypoint reached

    # Wait for robot position data to be available
    while limo.position_x == 0 and limo.position_z == 0:
        rospy.sleep(0.1)  # Wait 100ms
        if rospy.is_shutdown():
            return
    startpos = [limo.position_x, limo.position_z, 0,0]

    #Get waypoints from Jonas Trajectory Files
    num_trial = 1 #Trial_number File for trajectory
    waypoints = getJonasWaypoints(num_trial)

    #Rastic Floor x:(-4500,3000) z:(-2000,3000)

    # Define waypoints (x, z) in mm, starting from current position
    # One waypoint = [x,y, heading_angle, angular_vel]
    # waypoints = np.array([
    #     [limo.position_x, limo.position_z, 0, 0],  # start position
    #     [-1500, 0, 0,0],
    #     [-1250, 0, 0,0],
    #     [-1000, 0, 0, 0],
    #     [-750, 0, 0,0],
    #     [-500,0,0,0],
    #     [-250, 0, 0,0],
    #     [0,0,0,0]
    # ]).T  # shape (4, N)

    # waypoints = np.array([
    #     [limo.position_x, limo.position_z, 0,0],  # start position
    #     [-1600, 900,0, 0],
    #     [-1000, 1800, 0, 0],
    #     [-170, 2200, 0, 0],
    #     [800, 1300, 0,0],
    #     [1300, 300, 0,0],
    #     [2100, 700, 0,0]
    # ]).T  # shape (4, N)

    # waypoints = np.array([
    #     [limo.position_x, limo.position_z, 0, 0],  # start position
    #     [-4000, 600, 0, 0],          # heading: 0° (+x direction)
    #     [-3800, 600, 0, 0],          # heading: 0°
    #     [-3600, 608, 0, 0],          # heading: 15°
    #     [-3401, 633, 0, 0],          # heading: 30°
    #     [-3205, 675, 0, 0],          # heading: 45°
    #     [-3014, 733, 0, 0],          # heading: 60°
    #     [-2830, 808, 0, 0],          # heading: 75°
    #     [-2655, 898, 0, 0],          # heading: 90°
    #     [-2490, 1003, 0, 0],         # heading: 105°
    #     [-2338, 1122, 0, 0],         # heading: 120°
    #     [-2199, 1254, 0, 0],         # heading: 135°
    #     [-2075, 1398, 0, 0],         # heading: 150°
    #     [-1967, 1553, 0, 0],         # heading: 165°
    #     [-1876, 1717, 0, 0],         # heading: 180°
    #     [-1803, 1891, 0, 0],         # heading: 195°
    #     [-1748, 2072, 0, 0],         # heading: 210°
    #     [-1713, 2259, 0, 0],         # heading: 225°
    #     [-1697, 2451, 0, 0],         # heading: 240°
    #     [-1701, 2646, 0, 0],         # heading: 255°
    #     [-1725, 2843, 0, 0],         # heading: 270° (+y direction)
    # ]).T  # shape (4, N)

    # #Return to startpos
    # waypoints = np.array([
    #     [limo.position_x, limo.position_z, 0,0],  # start position
    #     [-2500,0,0,0]
    # ]).T  # shape (4, N)

    # #Return to startpos
    # waypoints = np.array([
    #     [limo.position_x, limo.position_z, 0,0],  # start position
    #     [2100,700,0,0]
    # ]).T  # shape (4, N)

    #Plot
    plot = plotLimoPosition(waypoints)  
    plot_controls = plotControlSignals()


    # Initial state estimate [x, z, yaw, unused]
    state = np.array([[limo.position_x], [limo.position_z], [0.0], [0.0]])   #4x1 

    current_wp_idx = 0
    total_waypoints = waypoints.shape[1]

    drive_msg = AckermannDrive()
    drive_msg.steering_angle_velocity = 0.0
    drive_msg.speed = 0.0
    last_steer_angle = 0.0

    heading_vector = np.array([[math.cos(current_yaw)], [-math.sin(current_yaw)]])


    # Skip waypoints very close to current position
    while current_wp_idx < total_waypoints:
        dist = np.linalg.norm(waypoints[0:2, current_wp_idx] - state[0:2, 0])
        if dist < distance_threshold:   
            print(f"Skipping waypoint {current_wp_idx + 1} (too close)")
            current_wp_idx += 1
        else:
            break

    #Iterates through list of waypoints 
    while current_wp_idx < total_waypoints:
        target_wp = waypoints[:, current_wp_idx:current_wp_idx+1]

        # PID error initialization
        error_steer_prev = np.cross(heading_vector.T, (target_wp[:2] - state[:2]).T)[0]
        #error_steer_prev = np.dot(heading_vector.T, (target_wp[:2] - state[:2]))[0, 0]
        error_steer_int = 0
        error_vel_prev = np.dot(heading_vector.T, (target_wp[:2] - state[:2]))[0, 0]
        error_vel_int = 0


        # The steering angle ranges from -0.7 to 0.7 radians
        # Clip to ensure we fall within that range
        # u_steer = max(min(u_steer,0.7),-0.7)
        MAX_STEER_ANGLE = 0.7  # radians (~30 degrees), tune as needed
        U_VEL_MAX = 0.7 
        steer_alpha = 0.5      # smoothing factor, between 0 (full smooth) and 1 (no smoothing)
        prev_steer = 0.0       # initialize outside the loop

        while not rospy.is_shutdown():
            print(f"Next waypoint: ({target_wp[0,0]:.1f}, {target_wp[1,0]:.1f})")
            heading_vector = np.array([[np.cos(current_yaw)], [-np.sin(current_yaw)]])

            # Update current state from limo and current yaw from callback
            state[0, 0] = limo.position_x
            state[1, 0] = limo.position_z
            heading_dir = heading_vector

            # # PID control for steering and velocity
            u_steer, error_steer_prev, error_steer_int = pid.steerControl(state, target_wp, heading_dir, error_steer_prev, error_steer_int)
            #pass steering error to speedPID
            u_vel, error_vel_prev, error_vel_int = pid.speedControl(state, target_wp, heading_dir, error_vel_prev, error_vel_int, error_steer_prev)         
          
            # # Replace both PID calls with this single call
            # u_steer, u_vel, error_steer_prev, error_steer_int, error_vel_prev, error_vel_int = pid.coordinatedControl(
            # state, target_wp, error_steer_prev, error_steer_int, error_vel_prev, error_vel_int)

            # Clamp and smooth steering
            # The steering angle ranges from -0.7 to 0.7 radians
            # Clip to ensure we fall within that range
            # example: u_steer = max(min(u_steer,0.7),-0.7)
            u_steer = max(min(u_steer, MAX_STEER_ANGLE), -MAX_STEER_ANGLE)
            smooth_steer = u_steer
            # smooth_steer = steer_alpha * last_steer_angle + (1 - steer_alpha) * u_steer
            # last_steer_angle = drive_msg.steering_angle_velocity

            # Ensure minimum forward motion but cap max speed
            u_vel = max(min(u_vel, U_VEL_MAX), 0.15)

            print(f"[PID] u_vel: {u_vel:.3f}, vel_error: {error_vel_prev:.3f}")
            print(f"current_yaw: {math.degrees(current_yaw):.2f}, u_steer: {u_steer:.3f}, steer_error: {error_steer_prev:.3f}")

            # Publish commands
            drive_msg.steering_angle_velocity= smooth_steer
            drive_msg.speed = u_vel
            limo.pub.publish(drive_msg)

            #Printing
            dist_to_wp = np.linalg.norm(target_wp[0:2, 0] - state[0:2, 0])
            print(f"[{current_wp_idx+1}/{total_waypoints}] Pos: ({state[0,0]:.1f}, {state[1,0]:.1f}), "
                  f"Vel: {u_vel:.2f}, Steer: {drive_msg.steering_angle_velocity:.2f}, Dist: {dist_to_wp:.1f}\n")

            #checks if robot is near target
            near_target = is_near_target_circle((limo.position_x, limo.position_z),target_wp, distance_threshold)
            if near_target:
                print(f"Reached waypoint {current_wp_idx + 1}\n")
                current_wp_idx += 1
                break
            
            #update live plot
            plot(limo.position_x, limo.position_z, heading_dir)
            plot_controls(u_vel, smooth_steer, error_steer_prev)
                
            rospy.Rate(1 / dt).sleep()
            #rospy.Rate(100).sleep()

        
            #Bounds of rastic floor: x: (-4800,3000) , z: (-2000, 3000)
            if limo.position_x > 3000 or limo.position_x < -4800 or limo.position_z > 3000 or limo.position_z < -2000: 
                print("Limo out of range of motion capture system, setting velocity to 0")
                drive_msg.speed = 0
                drive_msg.steering_angle_velocity = 0
                limo.pub.publish(drive_msg)
                sys.exit(0)
            
                
    print("Reached all waypoints.")
    drive_msg.speed = 0
    drive_msg.steering_angle_velocity = 0
    limo.pub.publish(drive_msg)
    plt.ioff()  # Turn off interactive mode
    plt.show()  # Now this will block until window is closed

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
