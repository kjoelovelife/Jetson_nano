#!/usr/bin/env python

import os, sys, argparse, errno, yaml, time, datetime
import rospy, rospkg
from numpy import *
from jetbot_msgs.msg import WheelsCmd

class InverseKinematicsNode(object):
    def __init__(self):
        # Get node name and vehicle name
        self.package = "jetbot_ros"
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        #self.start = rospy.wait_for_message("/" + self.veh_name +"/jetson_camera/image/raw", Image)
        rospy.loginfo("{}  Initializing inference_model.py......".format(self.node_name))    

        # Set parameters using yaml file

        # Set local variable by reading parameters
        self.limit = 1.0

        # Prepare services

        # Setup the publisher and subscriber
        self.sub_car_cmd = rospy.Subscriber("~cmd_vel", Twist, self.car_cmd_callback)
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmd, queue_size=1)
        rospy.loginfo("[%s] Initialized.", self.node_name)
        
    def car_cmd_callback(self, msg_car_cmd):

        omega_r = (msg_car_cmd.linear.x - msg_car_cmd.angular.z)
        omega_l = (msg_car_cmd.linear.x + msg_car_cmd.angular.z)
        
        # limiting output to limit, which is 1.0
        u_r_limited = max(min(omega_r, self.limit), -self.limit)
        u_l_limited = max(min(omega_l, self.limit), -self.limit)

        # setup WheelsCmd()
        msg_wheels_cmd = WheelsCmd()
        msg_wheels_cmd.vel_right = u_r_limited        
        msg_wheels_cmd.vel_left = u_l_limited

        # Put the wheel commands in a message and publish
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

    def on_shutdown(self): 
        rospy.is_shutdown=True

if __name__ == '__main__':
    rospy.init_node('inverse_kinematics_twist_node', anonymous=False)
    inverse_kinematics_node = InverseKinematicsNode()
    rospy.on_shutdown(inverse_kinematics_node.on_shutdown)
    rospy.spin()
