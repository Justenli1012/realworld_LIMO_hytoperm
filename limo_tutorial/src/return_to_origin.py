#!/usr/bin/env python3
import time
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from cav_for_line_follower import CAV

def get_yaw_from_quaternion(orientation_q):
    """Convert quaternion to yaw (heading angle in radians)."""
    q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    euler = tf.transformations.euler_from_quaternion(q)
    return euler[2]

def main():
    # Create CAV instance after node init
    limo = CAV("limo780")

    # Variables to store pose data
    current_yaw = None

    # Callback to update yaw from PoseStamped
    def pose_callback(msg):
        nonlocal current_yaw
        current_yaw = get_yaw_from_quaternion(msg.pose.orientation)
        # Position is updated inside limo via its own subscriber

    # Subscribe to pose for yaw updates
    rospy.Subscriber('/vrpn_client_node/limo780/pose', PoseStamped, pose_callback)

    # Wait until we have received initial position and yaw
    print("Waiting for initial position and yaw...")
    rate_wait = rospy.Rate(20)
    while (limo.Receivedata == 0 or current_yaw is None) and not rospy.is_shutdown():
        rate_wait.sleep()

    print(f"Initial position: x={limo.position_x:.1f} mm, z={limo.position_z:.1f} mm")
    print(f"Initial yaw: {current_yaw:.3f} radians")

    # Set target point here (change as needed)
    target_x = 0.0
    target_z = 0.0

    distance_threshold = 100  # millimeters
    transmission_rate = 30
    dt = 1.0 / transmission_rate

    # Compute line coefficients for the line between current position and target
    A = limo.position_z - target_z
    B = target_x - limo.position_x
    C = limo.position_x * target_z - target_x * limo.position_z

    eprev_lateral = 0
    eint_lateral = 0

    rate = rospy.Rate(transmission_rate)

    while not rospy.is_shutdown():
        # Safety bounds check
        if (limo.position_x > 3000 or limo.position_x < -4800 or
            limo.position_z > 3000 or limo.position_z < -2000):
            print("❌ Limo out of range, stopping!")
            drive_msg = limo.control(0, 0, 0, 0, dt)[2]
            limo.pub.publish(drive_msg)
            break

        # Calculate distance to target
        dist_to_target = np.linalg.norm([target_x - limo.position_x, target_z - limo.position_z])
        print(f"Distance to target: {dist_to_target:.1f} mm")

        if dist_to_target < distance_threshold:
            print("✅ Reached target point, stopping limo.")
            drive_msg = limo.control(0, 0, 0, 0, dt)[2]
            limo.pub.publish(drive_msg)
            break

        # Calculate lateral error to line
        denom = np.sqrt(A**2 + B**2)
        e = -(A * limo.position_x + B * limo.position_z + C) / (denom + 1e-8)

        # Calculate heading and goal vectors
        heading_vec = np.array([np.cos(current_yaw), np.sin(current_yaw)])
        goal_vec = np.array([target_x - limo.position_x, target_z - limo.position_z])
        goal_norm = np.linalg.norm(goal_vec)
        goal_unit = goal_vec / goal_norm if goal_norm > 1e-8 else goal_vec

        alignment = np.dot(heading_vec, goal_unit)
        vel = 0.5 if alignment >= 0 else -0.5

        # PID control for steering and velocity
        eprev_lateral, eint_lateral, drive_msg = limo.control(e, vel, eprev_lateral, eint_lateral, dt)

        print(f"Error: {e:.3f} | Steering: {drive_msg.steering_angle:.3f} | Speed: {drive_msg.speed:.3f}")

        # Publish control commands
        limo.pub.publish(drive_msg)

        rate.sleep()

    # Ensure the robot stops at the end
    drive_msg = limo.control(0, 0, 0, 0, dt)[2]
    limo.pub.publish(drive_msg)
    print("Node finished cleanly.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
