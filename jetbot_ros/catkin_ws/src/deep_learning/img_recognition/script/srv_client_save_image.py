#!/usr/bin/env python

import os, sys, argparse, errno, yaml, time, select, termios, tty
import cv2, numpy 
import rospy, rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
from uuid import uuid1
from img_recognition.srv import save_action, save_actionResponse, select_label, select_labelResponse



class srv_client_save_image_action(object):

    def __init__(self):
        
        # node information
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]

        # Publications
        self.pub_car_cmd = rospy.Publisher("~cmd_vel", Twist, queue_size=1)

        # wait for service start
        rospy.loginfo("wait for service.Please launch [save_image.launch] in [object] ")
        self.start = rospy.wait_for_service("/" + self.veh_name +"/save_image/save_image_action")
        self.start = rospy.wait_for_service("/" + self.veh_name +"/save_image/select_label") 

        # set service client
        self.save_image_action = rospy.ServiceProxy("/" + self.veh_name + "/save_image/save_image_action",save_action)
        self.select_label = rospy.ServiceProxy("/" + self.veh_name + "/save_image/select_label",select_label)
       
        # car parameter
        read_gain = self.readParamFromFile_gain()
 
        # ros parameter
        self.label = rospy.get_param("/" +self.veh_name + "/save_image/label","default")

        # local parameter 
        self.picture = False
        self.all_label = self.readParamFromFile_label()

        # Done information
        rospy.loginfo("You can press [w/a/s/d/x/q/r/z/c] to control the ROSKY with parameter in [{}] ".format(self.getFilePath_gain(self.veh_name)))
        rospy.loginfo("Service Start! You can click [p] to save picture.")
        rospy.logwarn("You can click [space] to change the label.")
        rospy.loginfo("Don't forget check out the label now!")
        rospy.logwarn("Now your label is [{}]".format(self.label))
       
        # timer
        try:
            self.timer = rospy.Timer(rospy.Duration.from_sec(1),self.cb_timer)
        except:
            _on_shutdown = self.on_shutdown() 
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            _exit = sys.exit()    

    def call_srv_save_image_action(self,picture):    
        try :
            if picture == True :
                rospy.loginfo("Capturing picture now!")
            else:
                rospy.loginfo("Stop capturing!")       
            send_signal = self.save_image_action(picture)
            image_count = rospy.get_param("/" +self.veh_name + "/save_image/label_image_count","didn't get!")         
            rospy.loginfo("The [ {} ] images you have : {}".format(self.label,image_count))
        except rospy.ServiceException as e :
            print("Service call failed".format(e))

    def cb_timer(self,event):
        _label = rospy.get_param("/" + self.veh_name +"/save_image/label","default")
        if _label != self.label:
            self.label = _label
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print ""
            rospy.logwarn("Now your label is [{}]".format(self.label))
            rospy.loginfo("You always can click [space] to change the label.")
            rospy.sleep(0.5)

    def call_srv_select_label(self,label="default"):    
        try :         
            send_signal = self.select_label(label)
            #rospy.loginfo("Select the label : {}".format(label))
        except rospy.ServiceException as e :
            print("Service call failed".format(e))
            
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01) 
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def on_shutdown(self): 
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rospy.is_shutdown=True


    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def readParamFromFile_gain(self):
        # Check file existence
        fname = self.getFilePath_gain(self.veh_name)
        # Use default.yaml if file doesn't exsit
        if not os.path.isfile(fname):
            rospy.logwarn("[%s] %s does not exist. Using default.yaml." %(self.node_name,fname))
            fname = self.getFilePath_gain("default")
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" %(self.node_name, fname, exc))
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["keyboard_gain","keyboard_steerGain"]:
            if param_name == "keyboard_gain":
                self.keyboard_gain = yaml_dict.get(param_name)
            else:
                self.keyboard_steerGain = yaml_dict.get(param_name)

    def getFilePath_gain(self, name):
        rospack = rospkg.RosPack()
        return rospack.get_path('rosky_base')+'/config/baseline/calibration/kinematics/' + name + ".yaml"

    def readParamFromFile_label(self):
        # Check file existence
        fname = self.getFilePath_label()
        # Use default.yaml if file doesn't exsit
        if not os.path.isfile(fname):
            rospy.logwarn("You don't have the label! Please check out {}".format(fname))
            self.on_shutdown()
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" %(self.node_name, fname, exc))
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        return yaml_dict.keys() 

    def getFilePath_label(self):
        rospack = rospkg.RosPack()
        return rospack.get_path('object') + "/param/image_label.yaml"

    def change_label(self):
        _all_label = {}
        for index in range(0,len(self.all_label)):
            _all_label[index] = self.all_label[index]       
        label_index = int(input("\nPlease select you label(just type number).\n {} : ".format(_all_label)))
        if label_index in _all_label.keys():
            self.label = _all_label[label_index]
            print("You select [ {} ]".format( self.label ))
            send_signal = self.select_label(self.label)
        else:
            print("You select non-existent label index. Please click [ space ] try again.\n")
          

    def public_car_cmd(self,speed=(0,0,0,0)):
        car_cmd = Twist()
        speed = (self.keyboard_gain * speed[0],0,0, self.keyboard_steerGain * speed[3])
        car_cmd.linear.x = speed[0]
        car_cmd.linear.y = 0
        car_cmd.linear.z = 0
        car_cmd.angular.x = 0
        car_cmd.angular.y = 0
        car_cmd.angular.z = speed[3]
        self.pub_car_cmd.publish(car_cmd)

if __name__ == "__main__" :
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("srv_client_save_image",anonymous=False)
    srv_call_save_image_action = srv_client_save_image_action()
    moveBindings = {
        'w':(1,0,0,0),
        'e':(1,0,0,-1),
        'a':(0,0,0,1),
        'd':(0,0,0,-1),
        'q':(1,0,0,1),
        'x':(-1,0,0,0),
        'c':(-1,0,0,1),
        'z':(-1,0,0,-1),
        'W':(1,-1,0,0),
        'E':(1,0,0,0),
        'A':(0,1,0,0),
        'D':(0,-1,0,0),
        'Q':(1,1,0,0),
        'X':(-1,0,0,0),
        'C':(-1,-1,0,0),
        'Z':(-1,1,0,0),
        's':(0,0,0,0),
        'S':(0,0,0,0)
    }
    rospy.spin()
    while(1):
        key = srv_call_save_image_action.getKey()
        if key == 'p' or key == 'P':
            srv_call_save_image_action.picture = not srv_call_save_image_action.picture
            call = srv_call_save_image_action.call_srv_save_image_action(srv_call_save_image_action.picture)
        elif key in moveBindings.keys():
            set_car_cmd = srv_call_save_image_action.public_car_cmd(moveBindings[key])
        elif key == ' ':
            change_label = srv_call_save_image_action.change_label()
        else:
            if key == '\x03':
                rospy.on_shutdown(srv_call_save_image_action.on_shutdown)
                break

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
    
