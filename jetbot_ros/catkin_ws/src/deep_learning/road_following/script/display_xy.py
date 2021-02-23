#!/usr/bin/env python
from __future__ import print_function
import sys , rospy ,cv2 ,time ,signal ,rospkg ,os.path , yaml , io ,thread, datetime
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from road_following.cfg import Draw_XY_Line_ParamConfig
from jetbot_msgs.msg import BoolStamped
from jetcam_ros.utils import bgr8_to_jpeg
from uuid import uuid1

class Display_XY_Node(object):
    def __init__(self):
        self.package = "road_following"
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        self.interval = {
            "timer": 0.5,
            "save": 1,
        }
        rospy.loginfo("[%s] Initializing......" %(self.node_name))

        # CV_bridge
        self.cv_bridge = CvBridge()
 
        # read/set ros parameter 
        self.folder_name = self.setup_parameter("~dataset", "dataset_xy")
        self.image_folder = self.check_folder(self.folder_name)

        # Set local parameter
        self.yaml_dict = self.read_parameter_file()
        self.img_width = self.yaml_dict["width"] 
        self.img_height = self.yaml_dict["height"]
        self.image_number = self.count_image_number(self.image_folder)

        self.is_shutdown = False
        self.update_framerate = False
        self.initialize = True
        self.line_parameter = {
            "X": 0,
            "Y": 0,
        }
        self.save_image_status = False
        self.save_image_done = False
             
        # Create service (for camera_calibration)
        #self.srv_set_camera_info = rospy.Service("~set_camera_info",SetCameraInfo,self.cbSrvSetCameraInfo)

        # start rqt_reconfigure
        self.reconfigure = Server(Draw_XY_Line_ParamConfig, self.set_line_parameter)

        # Setup Publisher and Subscriber
        self.has_published = False
        self.sub_img = rospy.Subscriber("~image/raw", Image, self.convert_image_to_cv2, queue_size=1)
        self.pub_img = rospy.Publisher("~image/raw/draw_xy_line", Image, queue_size=1)

        # Setup timer
        #timer = rospy.Timer(rospy.Duration.from_sec(self.interval["timer"]), self.cb_timer)

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def set_line_parameter(self, config, level):
        if self.initialize == True:
            for keys in self.line_parameter:
                config[keys] = self.line_parameter[keys]
                config["picture_interval"] = self.interval["save"]
                config["image_number"] = str(self.count_image_number(self.image_folder))
            self.initialize = False
        else:
            for keys in self.line_parameter:
                self.line_parameter[keys] = config[keys]
            self.interval["save"] = config["picture_interval"]
            self.save_image_status = config["save_image"]
            while(self.save_image_status):
                pass
            config["save_image"] = self.save_image_status
            config["image_number"] = str(self.count_image_number(self.image_folder))
        return config

    def convert_image_to_cv2(self, img_msg):
        try:
            # Conver your ROS Image messsage to opencv2
            img_cv2 = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8") 
        except CvBridgeError as e:
            print(e)
        self.draw_xy_line(img_cv2)
        if self.save_image_status == True:
            self.save_image(img_cv2)
            count = self.count_image_number(self.image_folder)
            if ( count - self.image_number) >= self.interval["save"]:
                self.save_image_status = False
                self.image_number = count

    def draw_xy_line(self, img_cv2):
        img = np.copy(img_cv2)
        x = self.line_parameter["X"]
        y = self.line_parameter["Y"]
        x = int( (x * self.img_width / 2) + (self.img_width / 2) )
        y = int( (y * self.img_height / 2) + (self.img_height / 2) )
        img = cv2.circle(img, (x,y), 8, (0, 255, 0), 3)
        img = cv2.circle(img, (self.img_width/2, self.img_height), 8, (0, 255, 0), 3)  
        img = cv2.line(img, (x,y), (self.img_width/2, self.img_height), (0, 0, 255), 3)
        img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.publisher(img_msg)

    def save_image(self, img_cv2):
        img_jpeg = bgr8_to_jpeg(img_cv2)
        img_name = "xy_%03d_%03d_%s" % (self.line_parameter["X"] * 50 + 50, self.line_parameter["Y"] * 50 + 50, uuid1())
        img_path = os.path.join(self.image_folder, img_name + '.jpg')
        with open(img_path, 'wb') as f:
            f.write(img_jpeg)
 
    def publisher(self, msg):
        self.pub_img.publish(msg)

    def read_parameter_file(self):
        file_name = self.get_file_path(self.veh_name)
        if not os.path.isfile(file_name):
            rospy.logwarn("[{}] {} does not exist. Using jetson_camera.yaml in package [jetson_camera/param]".format(self.node_name, file_name))
            file_name = self.get_file_path("jetson_camera")
        with open(file_name, "r") as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.MarkedYAMLError as exc:
                rospy.logfatal("[{}] Yaml syntax error. File: {}. Exc: {}".format(self.node_name, file_name, exc))
                rospy.signal_shutdown()
                return

    def get_file_path(self, name):
        rospack = rospkg.RosPack()
        return rospack.get_path("jetson_camera") + "/param/" + name + ".yaml"
 
    def count_image_number(self, folder_dir):
        count = 0
        for dir_path, dir_name, file_names in os.walk(folder_dir):
            for name in file_names:
                if name.endswith('jpg') or name.endswith('jpeg'):
                    count = count + 1
        return count

    def check_folder(self, name):
        rospack = rospkg.RosPack()
        folder = rospack.get_path(self.package) + "/image/" + name
        if not os.path.isdir(folder):
            time_format = "%Y_%m_%d_%H_%M_%S"
            now = datetime.datetime.now().strftime(time_format)
            rospy.logwarn("[{}] Can't find folder: {}.\nWill make [dataset_xy_{}] in {}.".format(self.node_name, folder, now, rospack.get_path(self.package)+"/image"))
            name = "dataset_xy" + "_" + now
            folder = rospack.get_path(self.package) + "/image/" + name
            os.makedirs(folder)
        return folder

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[{}] {} = {}".format(self.node_name, param_name, value))
        return value


    def onShutdown(self):
        rospy.loginfo("[%s] Closing camera." %(self.node_name))
        rospy.loginfo("[%s] Shutdown." %(self.node_name))
        rospy.sleep(0.5)
        rospy.is_shutdown=True
 

if __name__ == '__main__': 
    rospy.init_node('Display_XY_Node',anonymous=False)
    display_xy_node = Display_XY_Node()
    rospy.on_shutdown(display_xy_node.onShutdown)
    rospy.spin()
