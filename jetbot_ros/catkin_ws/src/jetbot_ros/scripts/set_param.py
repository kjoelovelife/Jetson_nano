#!/usr/bin/env python
# -*- coding: utf-8 -*

# Copyright (c) 2020, Joe Lin : kjoelovelife@gmail.com
# All rights reserved.
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


import sys, select, termios, tty

import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from jetbot_msgs.srv import SetValueRequest , SetValueResponse, SetValue
import rospkg , yaml , time , os 
import numpy as np

class Set_Param(object):
    def __init__(self):
        # Get node name and vehicle name
        self.node_name = rospy.get_name()      
        self.veh_name = self.node_name.split("/")[1]
        # Set parameters using yaml file
        self.readParamFromFile()

        # Set local variable by reading parameters
        self.gain = rospy.get_param( "/" + self.veh_name + "/gain", 1.0)
        self.trim = rospy.get_param("/" + self.veh_name + "/trim", 0.0)
        self.baseline = rospy.get_param("/" + self.veh_name + "/baseline", 0.1)
        self.radius = rospy.get_param("/" + self.veh_name + "/radius", 0.0318)
        self.k = rospy.get_param("/" + self.veh_name + "/k", 27.0)
        self.limit = rospy.get_param("/" + self.veh_name + "/limit", 1.0)
        self.motor_alpha = rospy.get_param("/" + self.veh_name + "/motor_alpha", -1.0)
        self.limit_max = 1.0
        self.limit_min = 0.0

        # Prepare services
        self.srv_set_gain = rospy.Service("~set_gain", SetValue, self.cbSrvSetGain)
        self.srv_set_trim = rospy.Service("~set_trim", SetValue, self.cbSrvSetTrim)
        self.srv_set_baseline = rospy.Service("~set_baseline", SetValue, self.cbSrvSetBaseline)
        self.srv_set_radius = rospy.Service("~set_radius", SetValue, self.cbSrvSetRadius)
        self.srv_set_k = rospy.Service("~set_k", SetValue, self.cbSrvSetK)
        self.srv_set_limit = rospy.Service("~set_limit", SetValue, self.cbSrvSetLimit)
        self.srv_set_motor_alpha = rospy.Service("~set_motor_alpha", SetValue, self.cbSrvSetMotorAlpha)
        self.srv_save = rospy.Service("~save_calibration", Empty, self.cbSrvSaveCalibration)

        # Setup the publisher and subscriber
        #self.sub_car_cmd = rospy.Subscriber("~car_cmd", Twist2DStamped, self.car_cmd_callback)
        #self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)
        #rospy.loginfo("[%s] Initialized.", self.node_name)
        #self.printValues()

    def readParamFromFile(self):
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use default.yaml if file doesn't exsit
        if not os.path.isfile(fname):
            rospy.logwarn("[%s] %s does not exist. Using default.yaml." %(self.node_name,fname))
            fname = self.getFilePath("default")

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" %(self.node_name, fname, exc))
                rospy.signal_shutdown()
                return

    def getFilePath(self, name):
        rospack = rospkg.RosPack()
        return rospack.get_path('jetbot_ros')+'/param/' + name + ".yaml"   


    def saveCalibration(self):
        # Write to yaml
        data = {
            "calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"),
            "gain": self.gain,
            "trim": self.trim,
            "baseline": self.baseline,
            "radius": self.radius,
            "k": self.k,
            "limit": self.limit,
            "motor_alpha": self.motor_alpha,
        }

        # Write to file
        file_name = self.getFilePath(self.veh_name)
        with open(file_name, 'w') as outfile:
            outfile.write(yaml.dump(data, default_flow_style=False))
        # Printout
        self.printValues()
        rospy.loginfo("[%s] Saved to %s" %(self.node_name, file_name))

    def cbSrvSaveCalibration(self, req):        
        self.saveCalibration()
        return EmptyResponse()

    def cbSrvSetGain(self, req):
        self.gain = req.value
        rospy.set_param( "/" + self.veh_name + "/gain", self.gain)
        self.printValues()
        return SetValueResponse()

    def cbSrvSetTrim(self, req):
        self.trim = req.value
        rospy.set_param( "/" + self.veh_name + "/trim", self.trim)
        self.printValues()
        return SetValueResponse()

    def cbSrvSetBaseline(self, req):
        self.baseline = req.value
        rospy.set_param( "/" + self.veh_name + "/baseline", self.baseline)
        self.printValues()
        return SetValueResponse()

    def cbSrvSetRadius(self, req):
        self.radius = req.value
        rospy.set_param( "/" + self.veh_name + "/radius", self.radius)
        self.printValues()
        return SetValueResponse()

    def cbSrvSetK(self, req):
        self.k = req.value
        rospy.set_param( "/" + self.veh_name + "/k", self.k)
        self.printValues()
        return SetValueResponse()

    def cbSrvSetLimit(self, req):
        self.limit = self.setLimit(req.value)
        rospy.set_param( "/" + self.veh_name + "/limit", self.limit)
        self.printValues()
        return SetValueResponse()

    def cbSrvSetMotorAlpha(self, req):
        self.motor_alpha = req.value
        rospy.set_param( "/" + self.veh_name + "/motor_alpha", self.motor_alpha)
        self.printValues()
        return SetValueResponse()

    def setLimit(self, value):
        if value > self.limit_max:
            rospy.logwarn("[%s] limit (%s) larger than max at %s" % (self.node_name, value, self.limit_max))
            limit = self.limit_max
        elif value < self.limit_min:
            rospy.logwarn("[%s] limit (%s) smaller than allowable min at %s" % (self.node_name, value, self.limit_min))
            limit = self.limit_min
        else:
            limit = value
        return limit

    def printValues(self):
        rospy.loginfo("[%s] gain: %s trim: %s baseline: %s radius: %s k: %s limit: %s" % (self.node_name, self.gain, self.trim, self.baseline, self.radius, self.k, self.limit))


if __name__ == '__main__':
    rospy.init_node('set_param_node', anonymous=False)
    set_param_node = Set_Param()
    rospy.spin()
