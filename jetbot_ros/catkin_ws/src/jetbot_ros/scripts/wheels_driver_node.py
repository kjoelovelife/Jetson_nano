#!/usr/bin/env python
import os, sys, argparse, errno, yaml, time, datetime
import rospy, rospkg
from numpy import *
from jetbot_msgs.msg import WheelsCmd
from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String


class Wheels_Driver_Node(object):
	self.package = "jetbot_ros"
    self.node_name = rospy.get_name()
    self.veh_name = self.node_name.split("/")[1]
    rospy.loginfo("[{}]  Initializing jetbot_motors.py......".format(self.node_name))

	# setup motor controller
	self.motor_driver = Adafruit_MotorHAT(i2c_bus=1)

	self.motor_left_ID = 1
	self.motor_right_ID = 2

	self.motor_left = self.motor_driver.getMotor(self.motor_left_ID)
	self.motor_right = self.motor_driver.getMotor(self.motor_right_ID)

    # setup subscriber
	self.sub_msg = rospy.Subscriber("~wheels_cmd" , WheelsCmd, self.set_speed)

    # sets motor speed between [-1.0, 1.0]
    def set_speed(self, data):
	    max_pwm = 115.0

        if data.vel_left >= 0 :
			self.motor_left.run(Adafruit_MotorHAT.FORWARD)
		else:
			self.motor_left.run(Adafruit_MotorHAT.BACKWARD)

	    if data.vel_right >= 0 :
			self.motor_right.run(Adafruit_MotorHAT.FORWARD)
		else:
			self.motor_right.run(Adafruit_MotorHAT.BACKWARD)

	    data.vel_left = int(min(max(abs(data.vel_left * max_pwm), 0), max_pwm))
		data.vel_right = int(min(max(abs(data.vel_right * max_pwm), 0), max_pwm))
	
	    self.motor_left.setSpeed(data.vel_left)
		self.motor_right.setSpeed(data.vel_right)

    # stops all motors
    def all_stop():
	    self.motor_left.setSpeed(0)
	    self.motor_right.setSpeed(0)
	    self.motor_left.run(Adafruit_MotorHAT.RELEASE)
	    self.motor_right.run(Adafruit_MotorHAT.RELEASE)

    def on_shutdown(self): 
        rospy.loginfo("[{}] Close.".format(self.node_name))
        rospy.loginfo("[{}] shutdown.".format(self.node_name))
        rospy.loginfo("[{}] Now you can press [ctrl] + [c] to close this launch file.".format(self.node_name))
        rospy.sleep(1)
		self.all_stop()
        rospy.is_shutdown = True
        #sys.exit()

# initialization
if __name__ == '__main__':
    rospy.init_node("wheels_driver_node", anonymous=False)
    wheels_driver_node = Wheels_Driver_Node()
    rospy.on_shutdown(wheels_driver_node.on_shutdown)   
    rospy.spin()

