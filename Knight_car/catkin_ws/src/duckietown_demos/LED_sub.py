#!/usr/bin/env python
import RPi.GPIO as gpio
import rospy
from std_msgs.msg import String
temp = 0
flag = 0
gpio.setmode(gpio.BCM)

def callback(data):
    global temp,flag
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    D = float(data.data)
    if D-temp >= 1.0:
	rospy.loginfo("~switch~")
	temp = D

	if flag==0:
	    gpio.output(18,gpio.HIGH)
	    rospy.loginfo("HIGH")
	    flag = 1
	else:
	    gpio.output(18,gpio.LOW)
	    rospy.loginfo("LOW")
	    flag = 0
    
    
def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

if __name__ == '__main__':
    gpio.setup(18,gpio.OUT)
    listener()
