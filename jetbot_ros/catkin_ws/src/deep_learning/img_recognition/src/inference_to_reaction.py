#!/usr/bin/env python
import os, sys, argparse, errno, yaml, time, datetime
import rospy, rospkg
import torch, torchvision, cv2
import numpy as np
from geometry_msgs.msg import Twist
from img_recognition.msg import Inference
from cv_bridge import CvBridge, CvBridgeError
from jetcam_ros.utils import bgr8_to_jpeg


class Inference_To_Reaction(object):
    def __init__(self):
        self.package = "img_recognition"
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        rospy.loginfo("[{}]  Initializing inference_model.py......".format(self.node_name))
        self.start = rospy.wait_for_message("/" + self.veh_name +"/inference_model/inference", Inference)
     
        # local parameter
        self.confidence = {}
        self.inference_gain = {
                "linear_velocity": [1, 1, 1], # Vx, Vy, Vz
                "angular_velocity": [1, 1, 1], # Ax, Ay, Az
        } 

        # ros parameter
        self.confidence_threshold = self.setup_parameter("~confidence_threshold", 0.75) 

        # setup the subscriber
        self.sub_msg_inference = rospy.Subscriber("~inference", Inference, self.inference_analyze, queue_size=1)

        # setup the publisher
        self.pub_car_cmd = rospy.Publisher("~pub_car_cmd", Twist, queue_size=1)

    def inference_analyze(self, data):
        if data == None:
            pass
        else:
            zip_data = zip(data.labels, data.confidence)
            self.confidence = dict(zip_data)
            recognition = max(self.confidence, key=self.confidence.get)
            if self.confidence[recognition] > self.confidence_threshold: 
                _reaction = self.reaction(recognition)

    def reaction(self, recognition): 
        if recognition == "free":
            for key in self.inference_gain.keys():
                for index in range(len(self.inference_gain[key])):
                    self.inference_gain[key][index] = 1
        elif recognition == "blocked":
            for key in self.inference_gain.keys():
                for index in range(len(self.inference_gain[key])):
                    self.inference_gain[key][index] = 0
        else:
            for key in self.inference_gain.keys():
                for index in range(len(self.inference_gain[key])):
                    self.inference_gain[key][index] = 1
        #self.setup_parameter("~inference_gain", inference_gain)
        self.cb_car_cmd(self.inference_gain)

    def cb_car_cmd(self, inference_gain):
        car_cmd_msg = Twist()
        car_cmd_msg.linear.x = 1.0 * inference_gain["linear_velocity"][0]
        car_cmd_msg.angular.z = 1.0 * inference_gain["angular_velocity"][2]
        self.pub_msg(car_cmd_msg)
    
    def pub_msg(self, car_cmd_msg):
        self.pub_car_cmd.publish(car_cmd_msg)


    def on_shutdown(self): 
        rospy.loginfo("[{}] Close.".format(self.node_name))
        rospy.loginfo("[{}] shutdown.".format(self.node_name))
        rospy.sleep(1)
        rospy.is_shutdown=True

    def setup_parameter(self, param_name, value):
        # value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == "__main__" :
    rospy.init_node("img_inference_to_reaction", anonymous=False)
    inference_to_reaction_node = Inference_To_Reaction()
    rospy.on_shutdown(inference_to_reaction_node.on_shutdown)   
    rospy.spin()
