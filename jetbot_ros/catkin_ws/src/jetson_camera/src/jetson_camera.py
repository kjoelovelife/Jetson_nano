#!/usr/bin/env python
from __future__ import print_function
import sys , rospy ,cv2 ,time ,signal ,rospkg ,os.path , yaml , io ,thread
import numpy as np
import camera_info_manager
from std_msgs.msg import String , Header
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from cv_bridge import CvBridge, CvBridgeError
from jetbot_msgs.msg import BoolStamped
# jetson nano camera
from jetcam_ros.csi_camera import CSICamera
from jetcam_ros.utils import bgr8_to_jpeg

class CameraNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        self.package = "jetson_camera"
        rospy.loginfo("[%s] Initializing......" %(self.node_name))

        # set jetson_camera 
        self.device = rospy.get_param("~capture_device",0)
        self.capture_fps = rospy.get_param("~capture_fps",60)
        self.capture_width = rospy.get_param("~capture_width",1280)
        self.capture_height = rospy.get_param("~capture_height",720)
        self.width = rospy.get_param("~width",224)
        self.height = rospy.get_param("~height",224)
        self.capture_flip = rospy.get_param("~capture_flip",0)  
        
        # grap camera
        try:
            self.camera = CSICamera(device=self.device,width=self.width, height=self.height, capture_width=self.capture_width, capture_height=self.capture_height, capture_fps=self.capture_fps,capture_flip=self.capture_flip)
        except StopIteration:
            print("Do not detect the camera.Please check your camera...  ")
            pass


        # For intrinsic calibration
        rospack = rospkg.RosPack()
        self.config = self.setupParam("~config","baseline")
        self.cali_file_folder = rospack.get_path(self.package) + "/param/" + self.veh_name + "/.yaml"
        self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"

        self.has_published = False
        self.pub_img= rospy.Publisher("~image/compressed",CompressedImage,queue_size=1)
        self.sub_switch_high = rospy.Subscriber("~framerate_high_switch", BoolStamped, self.cbSwitchHigh, queue_size=1)
        self.stream = io.BytesIO()
 
       # self.camera.exposure_mode = 'off'
       # self.camera.awb_mode = 'off'

        self.is_shutdown = False
        self.update_framerate = False
        # Setup timer
        rospy.loginfo("[%s] Initialized." %(self.node_name))
        
        # Create service (for camera_calibration)
        self.srv_set_camera_info = rospy.Service("~set_camera_info",SetCameraInfo,self.cbSrvSetCameraInfo)

    def cbSwitchHigh(self, switch_msg):
        print(switch_msg)
        if switch_msg.data and self.framerate != self.framerate_high:
            self.framerate = self.framerate_high
            self.update_framerate = True
        elif not switch_msg.data and self.framerate != self.framerate_low:
            self.framerate = self.framerate_low
            self.update_framerate = True
 
    def startCapturing(self):
        rospy.loginfo("[%s] Start capturing." %(self.node_name))
        while not self.is_shutdown and not rospy.is_shutdown():
            #gen =  self.grabAndPublish(self.stream,self.pub_img)
            try:
                image = self.camera.read()
                self.stream = np.array(bgr8_to_jpeg(image)).tostring()
                stamp = rospy.Time.now()
                stream_data = self.stream
                # Generate compressed image
                image_msg = CompressedImage()
                image_msg.format = "jpeg"
                image_msg.data = stream_data

                image_msg.header.stamp = stamp
                image_msg.header.frame_id = self.frame_id
                self.pub_img.publish(image_msg)
            
                if not self.has_published:
                    rospy.loginfo("[%s] Published the first image." %(self.node_name))
                    self.has_published = True

                rospy.sleep(rospy.Duration.from_sec(0.001))
            except StopIteration:
                pass
            #print("updating framerate")
            self.capture_fps =  self.capture_fps
            self.update_framerate=False

        #self.camera.close()
        rospy.loginfo("[%s] Capture Ended." %(self.node_name))

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        rospy.loginfo("[%s] Closing camera." %(self.node_name))
        self.is_shutdown=True
        rospy.loginfo("[%s] Shutdown." %(self.node_name))
        rospy.sleep(0.5)


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

if __name__ == '__main__': 
    rospy.init_node('camera',anonymous=False)
    camera_node = CameraNode()
    rospy.on_shutdown(camera_node.onShutdown)
    thread.start_new_thread(camera_node.startCapturing, ())
    rospy.spin()
