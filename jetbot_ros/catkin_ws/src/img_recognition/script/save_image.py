#!/usr/bin/env python

import os, sys, argparse, errno, yaml, time
import cv2, numpy 
import rospy, rospkg, threading
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from uuid import uuid1
from img_recognition.srv import save_action, save_actionResponse, select_label, select_labelResponse, picture_interval, picture_intervalResponse
from jetcam_ros.utils import bgr8_to_jpeg


class Save_Image_Node():
    ####
    # param : label, picture_interval
    # service : save_image_action, select_label, picture_interval
    ####  
    def __init__(self):
        # node information
        self.folder = "img_recognition"
        rospy.sleep(2)
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        rospy.loginfo("{}  Initializing......".format(self.node_name))

        # set local param
        self.save_action_status = False

        # read label
        self.label = self.setup_parameter("~label","default")
        self.yaml_dict = self.read_param_from_file()
        rospy.loginfo("Your image label : {} ".format(self.label))
        rospy.loginfo("If your label is wrong, please change the label.")

        # configure subscriber
        self.sub_msg = rospy.Subscriber("~image/raw", Image, self.convert_image_to_cv2, queue_size=1)
        
        # setup ros param
        self.picture_interval = self.setup_parameter("~picture_interval",0.5)
        if self.label in self.yaml_dict :
            self.label_image_count = self.setup_parameter("~label_image_count",self.yaml_dict[self.label]) 

        # Prepare ros services
        self.srv_save_image = rospy.Service("~save_image_action",save_action, self.cb_srv_save_image)
        self.srv_select_label = rospy.Service("~select_label",select_label, self.cb_srv_select_label)
        self.srv_picture_interval = rospy.Service("~picture_interval",picture_interval, self.cb_srv_picture_interval)

        # CV_bridge
        self.bridge = CvBridge()

        # directory for image save
        #self.path = os.path.abspath(os.path.join(os.path.dirname(__file__),os.pardir)) # ~/ROSKY/catkin_ws/src/self.folder , use system setting
        self.path = self.getFilePath(self.label)

        # timer
        self.save_image_timer = rospy.Timer(rospy.Duration.from_sec(self.picture_interval),self.cb_save_image_timer)
      
        # done information
        rospy.loginfo("Remember checkout the image size(width=224,height=224).")
        rospy.loginfo("You can use service with [srv_client_save_image.py] to start collecting your data!()")
        rospy.loginfo("The label(folder) and image you have :")
        rospy.loginfo(sorted(self.yaml_dict.items(), key=lambda x:x[0]))

    def getFilePath(self , name):
        rospack = rospkg.RosPack()
        return rospack.get_path(self.folder) + "/image/" + name   

    def convert_image_to_cv2(self,img_msg):
        try:
            # Convert your ROS Image ssage to opencv2
            self.cv2_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e) 

    def cb_srv_save_image(self, request):
        if request.value == True:
            self.save_action_status = True
            rospy.loginfo("Save action Start!!")
        else:
            self.save_action_status = False
            self.yaml_dict = self.read_param_from_file()
            if self.label in self.yaml_dict :
                self.label_image_count = self.yaml_dict[self.label]
                set_param = rospy.set_param("~label_image_count",self.label_image_count) 
                print("")
                rospy.loginfo("Save action Stop!!")
                rospy.loginfo("The label(folder) and image you have :")
                rospy.loginfo(sorted(self.yaml_dict.items(), key=lambda x:x[0]))

        return save_actionResponse()

    def cb_srv_select_label(self, request):
        if request.value in self.yaml_dict :
            rospy.set_param("~label",request.value)
            self.label = request.value
            self.path = self.getFilePath(self.label)
            rospy.loginfo("You select the label : [{}]".format(self.label))
            rospy.loginfo("Now your image will save in [{}]".format(self.path))
        else:
           rospy.loginfo("You don't have the label(folder) [{}] .".format(request.value))
           rospy.loginfo("Please use  {}  to make the label(floder) for saving data ".format(rospkg.RosPack().get_path(self.folder) + "/script/mkdir.py"))
        return select_labelResponse()   

    def cb_srv_picture_interval(self, request):
        self.picture_interval = request.value
        rospy.loginfo("Picture interval set {} seconds .".format(request.value))
        return picture_intervalResponse() 

    def cb_save_image_timer(self,event):
        if self.save_action_status == True:
            jpeg_img = bgr8_to_jpeg(self.cv2_img)
            self.save_img(jpeg_img)

    def save_img(self,img):
        img_path = os.path.join(self.path, str(uuid1())+ '.jpg')
        with open(img_path, 'wb') as f:
            f.write(img)
        rospy.loginfo("save image in {} ".format(self.path))


    def on_shutdown(self): 
        self.write_to_file()
        rospy.loginfo("{} Close.".format(self.node_name))
        rospy.loginfo("{} shutdown.".format(self.node_name))
        rospy.sleep(1)
        rospy.is_shutdown=True
        try:
            sys.exit(0)
        except:
            rospy.loginfo("Now you can press [ctrl] + [c] to shutdwon the lauch file.")

    def read_param_from_file(self):
        fname = rospkg.RosPack().get_path(self.folder) + "/param/image_label.yaml"
        folder = os.listdir(rospkg.RosPack().get_path(self.folder)+"/image")
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                for key in list(yaml_dict.keys()):
                    if key not in folder:
                        rospy.loginfo("Please checkout folder [image] and label in [/param/image_label.yaml]. They are different.")
                        rospy.loginfo("save_image.py will shutdown. Please shutdown the launch file after it(jetson_camera.py still runnung).")
                        sys.exit()
                if self.label not in yaml_dict.keys() :
                    rospy.logerr("Your /rosky/save_image/label [{}] is wrong.".format(self.label))
                    rospy.logerr("You can only type {}".format(sorted(yaml_dict)))
                    sys.exit()                         
                else:
                    pass
            except yaml.YAMLError as exc:
                print(" YAML syntax  error. File: {}".format(fname))
        if yaml_dict != None: 
            for label_name in yaml_dict:
                image_count = 0
                for dir_path, dir_names, file_names in os.walk(self.getFilePath(label_name)+ "/"):                   
                    for image in file_names:
                        if image.endswith('jpg') or image.endswith('jpeg') :
                            image_count += 1  
                yaml_dict[label_name] = image_count
                return yaml_dict
        else:
            rospy.loginfo("Please use  {}  to make the floder for saving data ".format(rospkg.RosPack().get_path(self.folder) + "/script/mkdir.py"))
            self.on_shutdown()

    def write_to_file(self):
        fname = rospkg.RosPack().get_path(self.folder) + "/param/image_label.yaml"
        self.yaml_dict = self.read_param_from_file()
        with open(fname, 'w') as outfile:
            outfile.write(yaml.dump(self.yaml_dict, default_flow_style=False))

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == "__main__" :
    rospy.init_node("save_image",anonymous=False)
    save_image_node = Save_Image_Node()
    rospy.on_shutdown(save_image_node.on_shutdown)
    rospy.spin()
    
    
