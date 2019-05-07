#!/usr/bin/env python
import rospy
import rospkg
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
import os.path
import yaml

class CamInfoReader(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        # Load parameters
        # self.pub_freq       = self.setupParam("~pub_freq",1.0)
        self.config         = self.setupParam("~config","baseline")
        self.cali_file_name = self.setupParam("~cali_file_name","default")
        self.image_type = self.setupParam("~image_type", "compressed")

        # Setup publisher
        self.pub_camera_info = rospy.Publisher("~camera_info",CameraInfo,queue_size=1)
        # Get path to calibration yaml file
        rospack = rospkg.RosPack()
        self.cali_file = rospack.get_path('duckietown') + "/config/" + self.config + "/calibration/camera_intrinsic/" +  self.cali_file_name + ".yaml" 
        self.camera_info_msg = None

        # Load calibration yaml file
        if not os.path.isfile(self.cali_file):
            rospy.logwarn("[%s] Can't find calibration file: %s.\nUsing default calibration instead." %(self.node_name,self.cali_file))
            self.cali_file = rospack.get_path('duckietown') + "/config/" + self.config + "/calibration/camera_intrinsic/default.yaml" 

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # Print out and prepare message
        rospy.loginfo("[%s] Using calibration file: %s" %(self.node_name,self.cali_file))
        self.camera_info_msg = self.loadCameraInfo(self.cali_file)
        self.camera_info_msg.header.frame_id = rospy.get_namespace() + "camera_optical_frame"
        rospy.loginfo("[%s] CameraInfo: %s" %(self.node_name,self.camera_info_msg))
        # self.timer_pub = rospy.Timer(rospy.Duration.from_sec(1.0/self.pub_freq),self.cbTimer)

        img_type = CompressedImage if self.image_type == "compressed" else Image
        typemsg = "CompressedImage" if self.image_type == "compressed" else "Image"
        rospy.logwarn("[%s] ==============%s",self.node_name, typemsg)
        self.sub_img_compressed = rospy.Subscriber("~compressed_image",img_type,self.cbCompressedImage,queue_size=1)
    
    def cbCompressedImage(self,msg):
        if self.camera_info_msg is not None:
            self.camera_info_msg.header.stamp = msg.header.stamp
            self.pub_camera_info.publish(self.camera_info_msg)

    # def cbTimer(self,event):
    #     self.camera_info_msg.header.stamp = rospy.Time.now()
    #     self.pub_camera_info.publish(self.camera_info_msg)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def loadCameraInfo(self, filename):
        stream = file(filename, 'r')
        calib_data = yaml.load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__': 
    rospy.init_node('cam_info_reader',anonymous=False)
    node = CamInfoReader()
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
