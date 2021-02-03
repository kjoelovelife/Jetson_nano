#!/usr/bin/env python
import os, sys, argparse, errno, yaml, time, datetime
import rospy, rospkg
import numpy as np
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
from img_recognition.msg import Inference as img_recognition_inference_msg
from road_following.msg import Inference as road_following_inference_msg
from road_following.cfg import PID_ControlConfig
from road_following.srv import save_action, save_actionResponse
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server

class Inference_To_Reaction(object):
    def __init__(self):
        self.package = "self_driving"
        self.road_following = "road_following"
        self.img_recognition = "img_recognition"
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        rospy.loginfo("[{}]  Initializing self_driving_inference_to_reaction.py......".format(self.node_name))
        #_start = rospy.wait_for_message("/" + self.veh_name +"/self_driving_inferece_model/road_following_inference", road_following_inference_msg)
        #_start = rospy.wait_for_message("/" + self.veh_name +"/self_driving_inferece_model/img_recognition_inference", img_recognition_inference_msg)
    
        # ros parameter
        self.pid_parameter = self.read_param_from_file(package=self.road_following, folder="param", file_name=(self.veh_name + "_pid.yaml"))
        for param_name in ["speed_gain", "steering_gain", "steering_kd", "steering_bias"]:
            param_value = self.pid_parameter[param_name]
            self.setup_parameter("~" + param_name, param_value)
        self.confidence_threshold = self.setup_parameter("~confidence_threshold", 0.75)

        # local parameter
        self.initialize = True
        self.want_labels = ["blocked", "free"]

        # setup the rqt_reconfigure 
        self.reconfigure = Server(PID_ControlConfig, self.set_pid_parameter)

        # setup the subscriber
        self.sub_msg_road_following_inference = Subscriber("~road_following_inference", road_following_inference_msg)
        self.sub_msg_img_recognition_inference = Subscriber("~img_recognition_inference", img_recognition_inference_msg)
        self.ts = ApproximateTimeSynchronizer([self.sub_msg_road_following_inference, self.sub_msg_img_recognition_inference], 1, 1, allow_headerless=True)
        self.ts.registerCallback(self.inference_analyze)
        #self.sub_msg_road_following_inference = rospy.Subscriber("~road_following_inference", road_following_inference_msg, self.inference_analyze, queue_size=1)
        #self.sub_msg_img_recognition_inference = rospy.Subscriber("~img_recognition_inference", img_recognition_inference_msg, self.inference_analyze, queue_size=1)


        # setup the publisher
        self.pub_car_cmd = rospy.Publisher("~cmd_vel", Twist, queue_size=1)

        # setup service
        self.srv_save_pid = rospy.Service("~save_pid", save_action, self.save_pid_parameter)


    def getFilePath(self, package, folder, file_name):
        rospack = rospkg.RosPack()
        return rospack.get_path(package) + "/" + folder + "/" + file_name  

    def read_param_from_file(self, package, folder, file_name):
        fname = self.getFilePath(package, folder, file_name)
        if not os.path.isfile(fname):
            if file_name == (self.veh_name + "_pid.yaml"):
                rospy.logwarn("[{}] {} does not exist. Using name \"default\" to load file.".format(self.node_name, fname))
                fname = self.getFilePath(package, folder, file_name="default_pid.yaml")
            else:
                rospy.logwarn("[{}] {} does not exist. Please check your file.".format(self.node_name, fname))
                self.on_shutdown()
                return
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                rospy.logerr("[{}] YAML syntax  error. File: {}".format(self.node_name, fname))
        return yaml_dict

    def set_pid_parameter(self, config, level):
        if self.initialize == True:
            for keys in self.pid_parameter:
                config[keys] = self.pid_parameter[keys]
            self.initialize = False
        else:
            for keys in self.pid_parameter:
                self.pid_parameter[keys] = config[keys]
            if config["save_parameter"]:
                self.save_pid_parameter(package=self.road_following, folder="param", file_name=(self.veh_name + "_pid.yaml"))
            config["save_parameter"] = False
        return config

    def inference_analyze(self, msg_road_following, msg_img_recognition):

        ## road following
        angle = msg_road_following.angle
        angle_last = msg_road_following.angle_last
        pid = angle * self.pid_parameter["steering_gain"] + (angle - angle_last) * self.pid_parameter["steering_kd"]
        steering_value = pid + self.pid_parameter["steering_bias"]

        ## img_recognition
        recognition = None
        if msg_img_recognition == None:
            pass
        else:
            zip_data = zip(msg_img_recognition.labels, msg_img_recognition.confidence)
            self.confidence = dict(zip_data)
            recognition = max(self.confidence, key=self.confidence.get)

        ## Publish twist 
        twist = Twist()
        if self.confidence[recognition] > self.confidence_threshold: 
            twist.linear.x = {
                "free": self.pid_parameter["speed_gain"],
                "blocked": 0,

            }.get(recognition, 0) 
        else:
            pass
        twist.angular.z = steering_value 
        self.pub_car_cmd.publish(twist)

    def srv_save_pid(self, request):
        self.save_pid_parameter(package=self.road_following, folder="param", file_name=(self.veh_name + "_pid.yaml"))
        return save_actionResponse

    def save_pid_parameter(self, package, folder, file_name):
        fname = rospkg.RosPack().get_path(package) + "/" + folder + "/" + file_name
        with open(fname, 'w') as outfile:
            outfile.write(yaml.dump(self.pid_parameter, default_flow_style=False))
        rospy.loginfo("[{}] Save parameter in {}.".format(self.node_name, fname))

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
    rospy.init_node("self_driving_inference_to_reaction", anonymous=False)
    inference_to_reaction_node = Inference_To_Reaction()
    rospy.on_shutdown(inference_to_reaction_node.on_shutdown)   
    rospy.spin()
