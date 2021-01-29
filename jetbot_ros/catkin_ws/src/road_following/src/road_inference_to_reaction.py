#!/usr/bin/env python
import os, sys, argparse, errno, yaml, time, datetime
import rospy, rospkg
import numpy as np
from road_following.msg import Inference
from road_following.cfg import PID_ControlConfig
from road_following.srv import save_action, save_actionResponse
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server

class Inference_To_Reaction(object):
    def __init__(self):
        self.package = "road_following"
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        rospy.loginfo("[{}]  Initializing road_inference_to_reaction.py......".format(self.node_name))
        self.start = rospy.wait_for_message("/" + self.veh_name +"/road_model_inference/inference", Inference)
    
        # ros parameter
        self.pid_parameter = {
            "speed_gain": 0,
            "steering_gain": 0.2,
            "steering_kd": 0,
            "steering_bias": 0,

        }
        _setup_parameter =  self.read_param_file()

        # local parameter
        self.initialize = True

        # setup the rqt_reconfigure 
        self.reconfigure = Server(PID_ControlConfig, self.set_pid_parameter)

        # setup the subscriber
        self.sub_msg_inference = rospy.Subscriber("~inference", Inference, self.inference_analyze, queue_size=1)

        # setup the publisher
        self.pub_car_cmd = rospy.Publisher("~cmd_vel", Twist, queue_size=1)

        # setup service
        self.srv_save_pid = rospy.Service("~save_pid", save_action, self.save_pid_parameter)

    def read_param_file(self):
        fname = rospkg.RosPack().get_path(self.package) + "/param/" + self.veh_name + "_pid.yaml"
        if not os.path.isfile(fname):
            rospy.logwarn("[{}] {} does not exist. Using default_pid.yaml to load parameter.".format(self.node_name, fname))
            fname = rospkg.RosPack().get_path(self.package) + "/param/default_pid.yaml"
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                rospy.logfatal("[{}] YAML syntax error. File: {} fname. Exc: {}".format(self.node_name, fname, exc))
                rospy.signal_shutdown()
                return
        if yaml_dict is not None:
            for param_name in ["speed_gain", "steering_gain", "steering_kd", "steering_bias"]:
                param_value = yaml_dict.get(param_name)
                self.pid_parameter[param_name] = self.setup_parameter("~" + param_name, param_value)
        else:
            return 

    def set_pid_parameter(self, config, level):
        if self.initialize == True:
            for keys in self.pid_parameter:
                config[keys] = self.pid_parameter[keys]
            self.initialize = False
        else:
            for keys in self.pid_parameter:
                self.pid_parameter[keys] = config[keys]
        return config

    def inference_analyze(self, data):
        angle = data.angle
        angle_last = data.angle_last
        pid = angle * self.pid_parameter["steering_gain"] + (angle - angle_last) * self.pid_parameter["steering_kd"]
        steering_value = pid + self.pid_parameter["steering_bias"]
        twist = Twist()
        twist.linear.x = self.pid_parameter["speed_gain"]
        twist.angular.z = steering_value 
        self.pub_msg(twist)
    
    def pub_msg(self, twist):
        self.pub_car_cmd.publish(twist)

    def save_pid_parameter(self, request):
        fname = rospkg.RosPack().get_path(self.package) + "/param/" + self.veh_name + "_pid.yaml"
        if not os.path.isfile(fname):
            rospy.logwarn("[{}] {}_pid.yaml does not exist in folder [param]. Using default_pid.yaml to save parameter.".format(self.node_name, self.veh_name))
            fname = rospkg.RosPack().get_path(self.package) + "/param/default_pid.yaml"
        with open(fname, 'w') as outfile:
            outfile.write(yaml.dump(recording_now, default_flow_style=False))
        return save_actionResponse

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def on_shutdown(self): 
        rospy.loginfo("[{}] Close.".format(self.node_name))
        rospy.loginfo("[{}] shutdown.".format(self.node_name))
        rospy.sleep(1)
        rospy.is_shutdown=True


if __name__ == "__main__" :
    rospy.init_node("road_inference_to_reaction", anonymous=False)
    inference_to_reaction_node = Inference_To_Reaction()
    rospy.on_shutdown(inference_to_reaction_node.on_shutdown)   
    rospy.spin()
