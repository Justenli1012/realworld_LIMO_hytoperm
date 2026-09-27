#!/usr/bin/env python3
from pylimo import limo
import rospy
from ackermann_msgs.msg import AckermannDrive
import time
import numpy as np
#import pickle

class listener:
    def __init__(self):
        self.data = None

    def callback(self, data):
        self.data = data
        

    def listener(self):
        rospy.init_node("vs_listener", anonymous=True)
        rospy.Subscriber("vel_steer_limo780", AckermannDrive, self.callback)


if __name__ == '__main__':
    listener_ins = listener()
    limo= limo.LIMO()
    limo.EnableCommand()
    limo.SetMotionCommand(linear_vel=0, steering_angle=0)
    time.sleep(500)
    steering_angle = np.zeros(250)
    #imu_yaw =  np.zeros(250)
    iter = 0 
    previous_steering = -6000 #arbitrary number
    while True:
        listener_ins.listener()
        if previous_steering == listener_ins.data.steering_angle:
            print("Previous steering = current steering commands. It is possible motion capture stopped updating, please double check and rerun")
            limo.SetMotionCommand(linear_vel=0, steering_angle=0)
            break
        elif listener_ins.data is not None:
            listener_ins.data.speed
            limo.SetMotionCommand(linear_vel=listener_ins.data.speed, steering_angle=listener_ins.data.steering_angle)
            print("Velocity: ", listener_ins.data.speed, "Steering: ", listener_ins.data.steering_angle, "X_POSITION: ", limo.position
            iter += 1
            if iter > 5:
                previous_steering = listener_ins.data.steering_angle
                iter = 0
            time.sleep(0.1)
        else:
            print("[WARNING] Not receiving ROS messages")
            if iter>10:
                break
    # outputFile = 'steering.pickle'
    # data = {'imu_yaw':imu_yaw, 'steer_limo':steering_angle}
    # fw = open(outputFile, 'wb')
    # pickle.dump(data, fw)
    # fw.close()
