#!/usr/bin/env python3
# coding=UTF-8
import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive

class CAV():
    def __init__(self, node_name):
        self.node_name = node_name
        self.position_z = 0
        self.position_x = 0
        self.velocity = 0
        self.Receivedata = 0

        #PID values
        self.kp = -0.0015 
        self.ki = -0.000045
        self.kd = -0.0017  

        #construct node, subscribe and publish the corrsponding rostopics 
        #rospy.init_node("listen_pos", anonymous=True)
        self.sub = rospy.Subscriber('/vrpn_client_node/'+self.node_name+'/pose', PoseStamped, self.callback)
        self.pub = rospy.Publisher('vel_steer_'+ self.node_name, AckermannDrive,queue_size=10) #topic name = CAV_Data

    def callback(self, msg):
        self.position_z = msg.pose.position.z*1000
        self.position_x = msg.pose.position.x*1000
        self.Receivedata=1

    def publish_drive_command(self, speed, steering_angle_velocity):
        drive_msg = AckermannDrive()
        drive_msg.speed = speed
        drive_msg.steering_angle_velocity = steering_angle_velocity
        self.pub.publish(drive_msg)