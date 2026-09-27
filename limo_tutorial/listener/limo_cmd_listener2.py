#!/usr/bin/env python3
from pylimo import limo
import rospy
from ackermann_msgs.msg import AckermannDrive
import numpy as np

class listener:
    def __init__(self):
        self.data = None

    def callback(self, data):
        self.data = data

if __name__ == '__main__':
    rospy.init_node("vs_listener2", anonymous=True)

    listener_ins = listener()
    rospy.Subscriber("vel_steer_limo780", AckermannDrive, listener_ins.callback)

    robot = limo.LIMO()
    robot.EnableCommand()
    
    # Initialize with zero motion using angular_vel parameter
    robot.SetMotionCommand(linear_vel=0, angular_vel=0)

    rate = rospy.Rate(30)  # 30 Hz loop

    iter = 0
    previous_steering_velocity = None

    while not rospy.is_shutdown():
        if listener_ins.data is not None:
            linear_vel = listener_ins.data.speed
            # Use steering_angle_velocity directly as angular velocity
            angular_vel = listener_ins.data.steering_angle_velocity

            # Send commands using angular_vel parameter (4-wheel differential)
            robot.SetMotionCommand(linear_vel=linear_vel, angular_vel=angular_vel)

            print("Linear velocity: ", linear_vel, "Angular velocity: ", angular_vel)

            # Detect command stagnation
            if previous_steering_velocity == angular_vel:
                iter += 1
            else:
                iter = 0

            if iter > 10*50:  # e.g. 50 cycles = ~1.6s at 30Hz
                print("[WARNING] Command not changing — stopping LIMO for safety.")
                robot.SetMotionCommand(linear_vel=0, angular_vel=0)
                break

            previous_steering_velocity = angular_vel
            rate.sleep()

        else:
            print("[WARNING] Not receiving ROS messages")
            iter += 1
            rate.sleep()