#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
import numpy as np

class WheelsTrimmerNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.trim = self.setupParam("~trim",0.0)

        # Setup publishers
        self.pub_topic = rospy.Publisher("~trimmed_wheels_cmd", WheelsCmdStamped, queue_size=1)
        # Setup subscribers
        self.sub_topic = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)
        # Setup timer for parameter update
        rospy.loginfo("[%s] Trim: %s" %(self.node_name, self.trim))
        self.timer_trim = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbTrim)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbTrim(self,event):
        trim = rospy.get_param("~trim",self.trim)
        if self.trim != trim:
            self.trim = trim
            rospy.loginfo("[%s] Trim updated to: %s"%(self.node_name,self.trim))

    def cbWheelsCmd(self,msg):
        trimmed_cmd = WheelsCmdStamped()
        trimmed_cmd.vel_left = np.clip(msg.vel_left*(1.0-self.trim),-1.0,1.0)
        trimmed_cmd.vel_right = np.clip(msg.vel_right*(1.0+self.trim),-1.0,1.0)
        trimmed_cmd.header.stamp = msg.header.stamp
        self.pub_topic.publish(trimmed_cmd)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down."%(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('wheels_trimmer_node', anonymous=False)
    # Create the DaguCar object
    node = WheelsTrimmerNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
