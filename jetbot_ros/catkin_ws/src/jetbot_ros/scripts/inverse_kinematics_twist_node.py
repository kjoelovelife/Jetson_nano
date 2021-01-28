#!/usr/bin/env python

import os, sys, argparse, errno, yaml, time, datetime
import rospy, rospkg
from numpy import *
from geometry_msgs.msg import Twist
from jetbot_msgs.msg import WheelsCmd
from dynamic_reconfigure.server import Server
from jetbot_ros.cfg import Wheels_DriverConfig
from std_srvs.srv import Empty, EmptyResponse

class InverseKinematicsNode(object):
    def __init__(self):
        # Get node name and vehicle name
        self.package = "jetbot_ros"
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        #self.start = rospy.wait_for_message("/" + self.veh_name +"/jetson_camera/image/raw", Image)
        rospy.loginfo("{}  Initializing inference_model.py......".format(self.node_name))    

        # read and setup ros parameters using yaml_file
        self.yaml_dict = self.read_param_from_file(self.veh_name)

         # Set local variable by reading parameters
        self.wheels_driver_parameter = self.yaml_dict
        self.initialize = True
        self.limit = 1.0

        # setup rqt_reconfigure
        self.rqt_reconfigure = Server(Wheels_DriverConfig, self.set_wheels_driver_parameter)

        # Prepare services
        self.srv_save_parameter = rospy.Service("~save_parameter", Empty, self.cb_save_parameter)
        _start = rospy.wait_for_service( self.node_name + "/save_parameter")


        # Setup the publisher and subscriber
        self.sub_car_cmd = rospy.Subscriber("~cmd_vel", Twist, self.car_cmd_callback)
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmd, queue_size=1)
        rospy.loginfo("[%s] Initialized.", self.node_name)

        # setup timer
        #timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.cb_timer)

    def set_wheels_driver_parameter(self, config, level):
        ignore_parameters = ["groups", "PID_function", "save_parameter"]
        if self.initialize == True:
            for keys in config.keys():
                if not keys in ignore_parameters :
                    config[keys] = self.wheels_driver_parameter[keys]
            self.initialize = False
        else:
            for keys in config.keys():
                if not keys in ignore_parameters :
                    self.wheels_driver_parameter[keys] = config[keys]
            if config["save_parameter"]:
                self.write_to_file()
            config["save_parameter"] = False           
        return config

    #def cb_timer(self, event):
    #    if self.saving_parameter_status:
    #        rospy.ServiceProxy(self.node_name + "/save_parameter" , Empty)

    def car_cmd_callback(self, msg_car_cmd):

        k_r = self.wheels_driver_parameter["k"]
        k_l = self.wheels_driver_parameter["k"]

        k_r_inv = (self.wheels_driver_parameter["speed_gain"] + self.wheels_driver_parameter["trim"]) / k_r
        k_l_inv = (self.wheels_driver_parameter["speed_gain"] - self.wheels_driver_parameter["trim"]) / k_l

        msg_car_cmd.angular.z = msg_car_cmd.angular.z * self.wheels_driver_parameter["steering_gain"]


        omega_r = (msg_car_cmd.linear.x + 0.5 * msg_car_cmd.angular.z * self.wheels_driver_parameter["baseline"]) / self.wheels_driver_parameter["radius"]
        omega_l = (msg_car_cmd.linear.x - 0.5 * msg_car_cmd.angular.z * self.wheels_driver_parameter["baseline"]) / self.wheels_driver_parameter["radius"]
        
        # u_r = (gain + trim)(v + 0.5 * omega * b) / (r * k_r)
        u_r =  k_r_inv * omega_r
        u_l =  k_l_inv * omega_l

        # limiting output to limit, which is 1.0
        u_r_limited = max(min(u_r, self.limit), -self.limit)
        u_l_limited = max(min(u_l, self.limit), -self.limit)

        # setup WheelsCmd()
        msg_wheels_cmd = WheelsCmd()
        msg_wheels_cmd.vel_right = u_r_limited        
        msg_wheels_cmd.vel_left = u_l_limited

        # Put the wheel commands in a message and publish
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[{}] {} = {}".format(self.node_name, param_name, value))
        return value

    def read_param_from_file(self, name):
        fname = self.getFilePath(name)
        if not os.path.isfile(fname):
            rospy.logwarn("[{}] {} does not exist. Use default.yaml to load parameter.".format(self.node_name, fname))
            fname = self.getFilePath("default")
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                print(" YAML syntax  error. File: {}".format(fname))
        for keys in yaml_dict:
            self.setup_parameter("~" + keys, yaml_dict[keys])
        return yaml_dict

    def get_time(self):
        time_format = '%Y_%m_%d_%H_%M_%S'
        now = datetime.datetime.now().strftime(time_format)
        return now

    def getFilePath(self , name):
        rospack = rospkg.RosPack()
        return rospack.get_path(self.package) + "/param/" + name + ".yaml"   

    def cb_save_parameter(self, req):
        self.write_to_file()
        return EmptyResponse()

    def write_to_file(self):
        fname = self.getFilePath(self.veh_name)
        self.wheels_driver_parameter["calibration_time"] = self.get_time()
        with open(fname, 'w') as outfile:
            outfile.write(yaml.dump(self.yaml_dict, default_flow_style=False))
        rospy.loginfo("[{}] save parameter in {}".format(self.node_name, fname))

    def on_shutdown(self): 
        rospy.is_shutdown=True

if __name__ == '__main__':
    rospy.init_node('inverse_kinematics_twist_node', anonymous=False)
    inverse_kinematics_node = InverseKinematicsNode()
    rospy.on_shutdown(inverse_kinematics_node.on_shutdown)
    rospy.spin()
