#!/usr/bin/env python
import os, sys, argparse, errno, yaml, time, datetime
import rospy, rospkg
from numpy import *
from jetbot_msgs.msg import WheelsCmd
from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Wheels_Driver_Node(object):
	def __init__(self):
		self.package = "jetbot_ros"
		self.node_name = rospy.get_name()
		self.veh_name = self.node_name.split("/")[1]
		rospy.loginfo("[{}]  Initializing jetbot_motors.py......".format(self.node_name))
		
		# setup ros parameter
		self.motor_alpha = self.setup_parameter("~alpha", -1.0)
		
		## setup motor controller
		self.motor_driver = Adafruit_MotorHAT(i2c_bus=1)
		self.motor_left_ID = 1
		self.motor_right_ID = 2
		self.motor_left = self.motor_driver.getMotor(self.motor_left_ID)
		self.motor_right = self.motor_driver.getMotor(self.motor_right_ID)

		## setup subscriber
		self.sub_msg = rospy.Subscriber("~wheels_cmd" , WheelsCmd, self.set_speed)
		
		## setup parameter
		timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.cb_timer)
		
	def cb_timer(self, event):
		self.motor_alpha = rospy.get_param("~alpha", -1.0)

	# sets motor speed between [-1.0, 1.0]
	def set_speed(self, data):
		max_pwm = 115.0
		data.vel_left = data.vel_left * self.motor_alpha
		data.vel_right = data.vel_right * self.motor_alpha

		if data.vel_left > 0 :
			self.motor_left.run(Adafruit_MotorHAT.FORWARD)
		else:
			self.motor_left.run(Adafruit_MotorHAT.BACKWARD)
		if data.vel_right > 0 :
		    self.motor_right.run(Adafruit_MotorHAT.FORWARD)
		else:
			self.motor_right.run(Adafruit_MotorHAT.BACKWARD)
		
		data.vel_left = int(min(max(abs(data.vel_left * max_pwm), 0), max_pwm))
		data.vel_right = int(min(max(abs(data.vel_right * max_pwm), 0), max_pwm))
		
		self.motor_left.setSpeed(data.vel_left)
		self.motor_right.setSpeed(data.vel_right)
	
	# stops all motors
	def all_stop(self):
		self.motor_left.setSpeed(0)
		self.motor_right.setSpeed(0)
		self.motor_left.run(Adafruit_MotorHAT.RELEASE)
		self.motor_right.run(Adafruit_MotorHAT.RELEASE)
		
	def setup_parameter(self, param_name, default_value):
		value = rospy.get_param(param_name, default_value)
		# Write to parameter server for transparency
		rospy.set_param(param_name, value)
		rospy.loginfo("[{}] {} = {}".format(self.node_name, param_name, value))
		return value
	
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

