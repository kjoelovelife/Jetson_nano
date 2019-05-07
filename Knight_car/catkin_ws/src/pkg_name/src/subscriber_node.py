#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg

# Define callback function
def callback(msg):
	rospy.loginfo("I heard: %s" %(msg.data))

# Initialize the node with rospy
rospy.init_node('subscriber_node', anonymous=False)
# Create subscriber
subscriber = rospy.Subscriber("topic", String, callback)

rospy.spin() #Keeps the script for exiting