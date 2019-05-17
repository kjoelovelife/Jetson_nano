#!/usr/bin/env python
# MIT License
# Copyright (c) 2019 JetsonHacks
# See license
# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a 
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image

import rospy
import cv2
import io
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from picamera import PiCamera
from picamera.array import PiRGBArray
import time
import signal
import sys
import rospkg
import os.path
import yaml
from duckietown_msgs.msg import BoolStamped
import thread

# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1280x720 @ 60fps 
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen

class CameraNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" %(self.node_name))


        self.framerate_high = self.setupParam("~framerate_high",30.0)
        self.framerate_low = self.setupParam("~framerate_low",15.0)
        self.res_w = self.setupParam("~res_w",640)
        self.res_h = self.setupParam("~res_h",480)
        self.image_msg = CompressedImage()

#============================================================================================
        # setup camera_information
        self.framerate = self.framerate_high # default to high        
  
#============================================================================================
        # For intrinsic calibration
        rospack = rospkg.RosPack()
        self.config = self.setupParam("~config","baseline")
        self.cali_file_folder = rospack.get_path('duckietown') + "/config/" + self.config + "/calibration/camera_intrinsic/"
    
        self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"

        self.has_published = False
        self.pub_img= rospy.Publisher("~image/compressed",CompressedImage,queue_size=1)
        self.sub_switch_high = rospy.Subscriber("~framerate_high_switch", BoolStamped, self.cbSwitchHigh, queue_size=1)

        # Create service (for camera_calibration)
        self.srv_set_camera_info = rospy.Service("~set_camera_info",SetCameraInfo,self.cbSrvSetCameraInfo)

        self.stream = io.BytesIO()

#self.camera.exposure_mode = 'off'
       # self.camera.awb_mode = 'off'

        self.is_shutdown = False
        self.update_framerate = False
        # Setup timer
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbSwitchHigh(self, switch_msg):
        print switch_msg
        if switch_msg.data and self.framerate != self.framerate_high:
            self.framerate = self.framerate_high
            self.update_framerate = True
        elif not switch_msg.data and self.framerate != self.framerate_low:
            self.framerate = self.framerate_low
            self.update_framerate = True

    def grabAndPublish(self,stream,publisher):
        while not self.update_framerate and not self.is_shutdown and not rospy.is_shutdown(): 
            yield stream
            # Construct image_msg
            # Grab image from stream
            stamp = rospy.Time.now()
            stream.seek(0)
            stream_data = stream.getvalue()
            # Generate compressed image
            image_msg = CompressedImage()
            image_msg.format = "jpeg"
            image_msg.data = stream_data

            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            publisher.publish(image_msg)
                        
            # Clear stream
            stream.seek(0)
            stream.truncate()
            
            if not self.has_published:
                rospy.loginfo("[%s] Published the first image." %(self.node_name))
                self.has_published = True

            rospy.sleep(rospy.Duration.from_sec(0.001))

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        rospy.loginfo("[%s] Closing camera." %(self.node_name))
        self.is_shutdown=True
        rospy.loginfo("[%s] Shutdown." %(self.node_name))


    def cbSrvSetCameraInfo(self,req):
        # TODO: save req.camera_info to yaml file
        rospy.loginfo("[cbSrvSetCameraInfo] Callback!")
        filename = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"
        response = SetCameraInfoResponse()
        response.success = self.saveCameraInfo(req.camera_info,filename)
        response.status_message = "Write to %s" %filename #TODO file name
        return response

    def saveCameraInfo(self, camera_info_msg, filename):
        # Convert camera_info_msg and save to a yaml file
        rospy.loginfo("[saveCameraInfo] filename: %s" %(filename))
        file = open(filename, 'w')

        # Converted from camera_info_manager.py
        calib = {'image_width': camera_info_msg.width,
        'image_height': camera_info_msg.height,
        'camera_name': rospy.get_name().strip("/"), #TODO check this
        'distortion_model': camera_info_msg.distortion_model,
        'distortion_coefficients': {'data': camera_info_msg.D, 'rows':1, 'cols':5},
        'camera_matrix': {'data': camera_info_msg.K, 'rows':3, 'cols':3},
        'rectification_matrix': {'data': camera_info_msg.R, 'rows':3, 'cols':3},
        'projection_matrix': {'data': camera_info_msg.P,'rows':3, 'cols':4}}
        
        rospy.loginfo("[saveCameraInfo] calib %s" %(calib))

        try:
            rc = yaml.safe_dump(calib, file)
            return True
        except IOError:
            return False
#======================== need to modified ===================================================

    def gstreamer_pipeline (capture_width=self.res_w, capture_height=self.res_h, display_width=self.res_w, display_height=self.res_h, framerate=self.framerate , flip_method=2) :   
        rospy.loginfo("[%s] Start capturing." %(self.node_name))
        return ('nvarguscamerasrc ! ' 
        'video/x-raw(memory:NVMM), '
        'width=(int)%d, height=(int)%d, '
        'format=(string)NV12, framerate=(fraction)%d/1 ! '
        'nvvidconv flip-method=%d ! '
        'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
        'videoconvert ! '
        'video/x-raw, format=(string)BGR ! appsink'  %     (capture_width,capture_height,framerate,flip_method,display_width,display_height))

    def startCapturing(self):
        rospy.loginfo("[%s] Start capturing." %(self.node_name))
        while not self.is_shutdown and not rospy.is_shutdown():
            gen =  self.grabAndPublish(self.stream,self.pub_img)
            try:
                self.camera.capture_sequence(gen,'jpeg',use_video_port=True,splitter_port=0)
            except StopIteratio\\\n:
                pass
            print "updating framerate"
            self.camera.framerate = self.framerate
            self.update_framerate=False

        self.camera.close()
        rospy.loginfo("[%s] Capture Ended." %(self.node_name))

    def show_camera():
        # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
        print gstreamer_pipeline(flip_method=2)
        cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
        if cap.isOpened():
            window_handle = cv2.namedWindow('CSI Camera', cv2.WINDOW_AUTOSIZE)
            # Window 
            while cv2.getWindowProperty('CSI Camera',0) >= 0:
                ret_val, img = cap.read();
                cv2.imshow('CSI Camera',img)
                # This also acts as 
                keyCode = cv2.waitKey(30) & 0xff
                # Stop the program on the ESC key
                if keyCode == 27:
                   break
            cap.release()
            cv2.destroyAllWindows()
        else:
            print 'Unable to open camera'
   
if __name__ == '__main__':
    rospy.init_node('camera',anonymous=False)
    camera_node = CameraNode()
    rospy.on_shutdown(camera_node.onShutdown)
    thread.start_new_thread(camera_node.startCapturing, ())
    rospy.spin()
