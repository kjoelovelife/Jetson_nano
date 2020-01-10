#!/usr/bin/env python

# Copyright (c) 2020, Lin Wei-Chih
# All rights reserved.
#
# Developer : Lin Wei-Chih , kjoelovelife@gmail.com 
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import time
import math
from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist





# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
	max_pwm = 115.0
	speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

	if motor_ID == 1:
		motor = motor_left
	elif motor_ID == 2:
		motor = motor_right
	else:
		rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
		return
	
	motor.setSpeed(speed)

	if value > 0:
		motor.run(Adafruit_MotorHAT.FORWARD)
	else:
		motor.run(Adafruit_MotorHAT.BACKWARD)


# stops all motors
def all_stop():
	motor_left.setSpeed(0)
	motor_right.setSpeed(0)

	motor_left.run(Adafruit_MotorHAT.RELEASE)
	motor_right.run(Adafruit_MotorHAT.RELEASE)

def on_cmd_vel(data):
    global right_speed , left_speed , veh_name
    # get param
    if veh_name != "unnamed":
        gain = rospy.get_param('/' + veh_name + '/gain', 5.0)
        trim = rospy.get_param('/' + veh_name + '/trim', -0.05000000074505806)
        baseline = rospy.get_param('/' + veh_name + '/baseline', 0.12)
        radius = rospy.get_param('/' + veh_name + '/radius', 0.0725)
        k = rospy.get_param('/' + veh_name + '/k', 27.0)
        motor_alpha = rospy.get_param('/' + veh_name + '/motor_alpha',-1.0)
        limit = rospy.get_param('/' + veh_name + '/limit', 1.0)
        v_gain = rospy.get_param('/' + veh_name + '/v_gain', 0.41)
        omega_gain = rospy.get_param('/' + veh_name + '/steer_gain', 8.3)
        steer_angle_gain = rospy.get_param('/' + veh_name + '/steer_angle_gain', 1)
        simulated_vehicle_length = rospy.get_param('/' + veh_name + '/simulated_vehicle_length', 0.2)
        navigation_gain = rospy.get_param('/' + veh_name + '/navigation_gain', 1.0)

    else:
        gain = rospy.get_param('~gain', 1.0)
        trim = rospy.get_param('~trim', -0.05000000074505806)
        baseline = rospy.get_param('~baseline', 0.12)
        radius = rospy.get_param('~radius', 0.0725)
        k = rospy.get_param('~k', 27.0)
        motor_alpha = rospy.get_param("~motor_alpha",-1.0)
        limit = rospy.get_param('~limit', 1.0)
        v_gain = rospy.get_param("~speed_gain", 0.41)
        omega_gain = rospy.get_param("~steer_gain", 8.3)
        steer_angle_gain = rospy.get_param("~steer_angle_gain", 1)
        simulated_vehicle_length = rospy.get_param('~simulated_vehicle_length', 0.2)
        navigation_gain = rospy.get_param('~/navigation_gain', 1.0)

    # get cmd velocity
    twist = data
    Vx = twist.linear.x * v_gain * navigation_gain
    Vy = twist.linear.y
    Vw = twist.angular.z * omega_gain  #Vx / simulated_vehicle_length * math.tan( twist.angular.z * steer_angle_gain )



    # assuming same motor constants k for both motors
    k_r = k
    k_l = k

    # adjusting k by gain and trim
    k_r_inv = (gain + trim) #/ k_r
    k_l_inv = (gain - trim) #/ k_l
    omega_r = (Vx + 0.5 * Vw * baseline) #/ radius
    omega_l = (Vx - 0.5 * Vw * baseline) #/ radius

    # conversion from motor rotation rate to duty cycle
    # u_r = (gain + trim) (v + 0.5 * omega * b) / (r * k_r)
    u_r = omega_r * k_r_inv
    # u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
    u_l = omega_l * k_l_inv

    # limiting output to limit, which is 1.0 for the jetbot
    u_r_limited = max(min(u_r, limit), -limit)
    u_l_limited = max(min(u_l, limit), -limit)

    # set motor
    set_speed(motor_left_ID,   u_l_limited * motor_alpha)
    set_speed(motor_right_ID,  u_r_limited * motor_alpha) 
    
    #show information
    rospy.loginfo(" right_speed = {}  ".format(u_r_limited) )
    rospy.loginfo(" left_speed  = {}  ".format(u_l_limited) )
    #rospy.loginfo(" node_name = {}  ".format(rospy.get_name()) )
    #rospy.loginfo(" omega_l  = {}  ".format(omega_l) )
    #rospy.loginfo(" Vx = {}  ".format(Vx) )
    #rospy.loginfo(" Vw = {}  ".format(Vw) )
    

# initialization
if __name__ == '__main__':

        # get node information
        global veh_name
        node_name = rospy.get_name()      
        veh_name = node_name.split("/")[1]
        print(veh_name)

	# setup motor controller
	motor_driver = Adafruit_MotorHAT(i2c_bus=1)

	motor_left_ID = 1
	motor_right_ID = 2

	motor_left = motor_driver.getMotor(motor_left_ID)
	motor_right = motor_driver.getMotor(motor_right_ID)
        right_speed = 0.0
        left_speed = 0.0

	# stop the motors as precaution
	all_stop()

	# setup ros node
	rospy.init_node('jetbot_motors')
	
	rospy.Subscriber('/teleop/cmd_vel', Twist, on_cmd_vel)




	# start running
	rospy.spin()

	# stop motors before exiting
	all_stop()

