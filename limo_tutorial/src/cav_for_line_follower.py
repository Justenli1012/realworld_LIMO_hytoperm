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
        rospy.init_node("listen_pos", anonymous=True)
        self.sub = rospy.Subscriber('/vrpn_client_node/'+self.node_name+'/pose', PoseStamped, self.callback)
        self.pub = rospy.Publisher('vel_steer_'+ self.node_name, AckermannDrive,queue_size=10) #topic name = CAV_Data
        rospy.Rate(10)

    def callback(self, msg):
        self.position_z = msg.pose.position.z*1000
        self.position_x = msg.pose.position.x*1000
        self.Receivedata=1

    #calculates steering
    def control(self, e, v_ref, eprev_lateral, eint_lateral, dt):
        if (eprev_lateral*e<=0):
            eint_lateral = 0
        kp = self.kp
        ki = self.ki
        kd = self.kd

        [ref_steer, eprev_lateral, eint_lateral] = self.PIDController(e, eprev_lateral, eint_lateral, dt, kp, ki, kd)

        drive_msg = AckermannDrive()
        drive_msg.speed = v_ref
        drive_msg.steering_angle = self.steeringAngleToSteeringCommand(ref_steer)
        return eprev_lateral, eint_lateral, drive_msg

    #helper function for control()
    def steeringAngleToSteeringCommand(self,refAngle):
        y = 0.7*refAngle
        return y
    
    #helper function for control(), the PID controller
    def PIDController(self, e, prev_e, prev_int, delta_t, Kp, Ki, Kd):
        if e <= 1 and e>=-1:
            e_int = 0
        # integral of the error
        e_int = prev_int + e*delta_t

        # anti-windup - preventing the integral error from growing too much
        e_int = max(min(e_int,0.3),-0.3)

        # derivative of the error
        e_der = (e - prev_e)/delta_t

        # PID controller for omega
        u = Kp*e + Ki*e_int + Kd*e_der

        return u, e, e_int

