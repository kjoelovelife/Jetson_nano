#!/usr/bin/env python
import rospy
import cv2
import io
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

class CameraNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" %(self.node_name))
        # TODO: load parameters

        self.framerate = self.setupParam("~framerate",60.0)
        self.res_w = self.setupParam("~res_w",320)
        self.res_h = self.setupParam("~res_h",200)

        # self.img_low_framerate = self.setupParam("~img_low_framerate",30.0)
        # self.img_high_framerate = self.setupParam("~img_high_framerate",5.0)
        # self.img_low_res_w = self.setupParam("~img_low_res_w",320)
        # self.img_low_res_h = self.setupParam("~img_low_res_h",200)
        # self.img_high_res_w = self.setupParam("~img_high_res_w",640)
        # self.img_high_res_h = self.setupParam("~img_high_res_h",400)
        # self.uncompress = self.setupParam("~uncompress",False)

        # TODO: load camera info yaml file and publish CameraInfo
        self.pub_img= rospy.Publisher("~image/compressed",CompressedImage,queue_size=1)

        # if self.uncompress:
        #     self.pub_img_low = rospy.Publisher("~img_low/raw",Image,queue_size=1)
        #     self.pub_img_high= rospy.Publisher("~img_high/raw",Image,queue_size=1)    
        # else:
        #     self.pub_img_low = rospy.Publisher("~img_low/compressed",CompressedImage,queue_size=1)
        #     self.pub_img_high= rospy.Publisher("~img_high/compressed",CompressedImage,queue_size=1)
        
        self.has_published = False
        self.bridge = CvBridge()

        # Setup PiCamera
        self.stream = io.BytesIO()
        self.bridge = CvBridge()
        self.camera = PiCamera()
        self.camera.framerate = self.framerate
        self.camera.resolution = (self.res_w,self.res_h)

        # TODO setup other parameters of the camera such as exposure and white balance etc

        # Setup timer
        self.camera_capture = self.camera.capture_continuous(self.stream,'jpeg',use_video_port=True)
        self.timer_img_low = rospy.Timer(rospy.Duration.from_sec(1.0/self.framerate),self.cbTimer)
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbTimer(self,event):
        if not rospy.is_shutdown():
            self.camera_capture.next()
            self.grabAndPublish(self.stream,self.pub_img)
            # Maybe for every 5 img_low, change the setting of the camera and capture a higher res img and publish.

    def grabAndPublish(self,stream,publisher):
        # Grab image from stream
        stream.seek(0)
        img_data = stream.getvalue()

        if self.uncompress:
            # Publish raw image
            data = np.fromstring(img_data, dtype=np.uint8)
            image = cv2.imdecode(data, 1)
            image_msg = self.bridge.cv2_to_imgmsg(image)
        else:
            # Publish compressed image only
            image_msg = CompressedImage()
            image_msg.data = img_data
            image_msg.format = "jpeg"
            
        image_msg.header.stamp = rospy.Time.now()
        # Publish 
        publisher.publish(image_msg)
        # Clear stream
        stream.seek(0)
        stream.truncate()

        if not self.has_published:
            rospy.loginfo("[%s] Published the first image." %(self.node_name))
            self.has_published = True

    def onShutdown(self):
        self.camera.close()
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__': 
    rospy.init_node('camera_node',anonymous=False)
    camera_node = CameraNode()
    rospy.on_shutdown(camera_node.onShutdown)
    rospy.spin()
