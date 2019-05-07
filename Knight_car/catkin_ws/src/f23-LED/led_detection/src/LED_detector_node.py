#!/usr/bin/env python
import rospy
import time
from led_detection.LEDDetector import LEDDetector
from std_msgs.msg import Byte
from duckietown_msgs.msg import Vector2D, LEDDetection, LEDDetectionArray, LEDDetectionDebugInfo, BoolStamped
from sensor_msgs.msg import CompressedImage
from duckietown_utils.bag_logs import numpy_from_ros_compressed
import numpy as np

class LEDDetectorNode(object):
    def __init__(self):
        self.active = True # [INTERACTIVE MODE] Won't be overwritten if FSM isn't running, node always active 
        self.first_timestamp = 0
        self.capture_time = 0.97 # capture time
        self.capture_finished = True
        self.tinit = None
        self.trigger = False
        self.node_state = 0
        self.data = []
        
        self.node_name = rospy.get_name()
        self.pub_detections = rospy.Publisher("~raw_led_detection",LEDDetectionArray,queue_size=1)
        self.pub_debug = rospy.Publisher("~debug_info",LEDDetectionDebugInfo,queue_size=1)
        self.veh_name = rospy.get_namespace().strip("/")

        self.protocol = rospy.get_param("~LED_protocol")
        self.crop_rect_normalized = rospy.get_param("~crop_rect_normalized")
        self.capture_time = rospy.get_param("~capture_time")
        self.cell_size = rospy.get_param("~cell_size")
        self.continuous = rospy.get_param('~continuous', True) # Detect continuously as long as active
                                                               # [INTERACTIVE MODE] set to False for manual trigger
        self.frequencies = self.protocol['frequencies'].values()

        rospy.loginfo('[%s] Config: \n\t crop_rect_normalized: %s, \n\t capture_time: %s, \n\t cell_size: %s'%(self.node_name, self.crop_rect_normalized, self.capture_time, self.cell_size))

        if not self.veh_name:
            # fall back on private param passed thru rosrun
            # syntax is: rosrun <pkg> <node> _veh:=<bot-id>
            if rospy.has_param('~veh'):
                self.veh_name = rospy.get_param('~veh')
              
        if not self.veh_name:
            raise ValueError('Vehicle name is not set.')

        rospy.loginfo('[%s] Vehicle: %s'%(self.node_name, self.veh_name))
        self.sub_cam = rospy.Subscriber("camera_node/image/compressed",CompressedImage, self.camera_callback)
        self.sub_trig = rospy.Subscriber("~trigger",Byte, self.trigger_callback)
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped,self.cbSwitch)
        rospy.loginfo('[%s] Waiting for camera image...' %self.node_name)

    def cbSwitch(self, switch_msg): # active/inactive switch from FSM
        self.active = switch_msg.data
        if(self.active):
            self.trigger = True

    def camera_callback(self, msg):
        if not self.active:
            return

        float_time = msg.header.stamp.to_sec()
        debug_msg = LEDDetectionDebugInfo()

        if self.trigger:
            rospy.loginfo('[%s] GOT TRIGGER! Starting...')
            self.trigger = False
            self.data = []
            self.capture_finished = False
            rospy.loginfo('[%s] Start capturing frames'%self.node_name)
            self.first_timestamp = msg.header.stamp.to_sec()
            self.tinit = time.time()

        elif self.capture_finished:
            self.node_state = 0
            rospy.loginfo('[%s] Waiting for trigger...' %self.node_name)

        if self.first_timestamp > 0:
            # TODO sanity check rel_time positive, restart otherwise 
            rel_time = float_time - self.first_timestamp

            # Capturing
            if rel_time < self.capture_time:
                self.node_state = 1
                rgb = numpy_from_ros_compressed(msg)
                rospy.loginfo('[%s] Capturing frame %s' %(self.node_name, rel_time))
                self.data.append({'timestamp': float_time, 'rgb': rgb[:,:,:]})
                debug_msg.capture_progress = 100.0*rel_time/self.capture_time

            # Start processing
            elif not self.capture_finished and self.first_timestamp > 0:
                rospy.loginfo('[%s] Relative Time %s, processing' %(self.node_name, rel_time))
                self.node_state = 2
                self.capture_finished = True
                self.first_timestamp = 0
                self.sub_cam.unregister() # IMPORTANT! Explicitly ignore messages  
                                          # while processing, accumulates delay otherwise!
                self.send_state(debug_msg)
                self.process_and_publish()

        self.send_state(debug_msg) # TODO move heartbeat to dedicated thread

    def trigger_callback(self, msg):
        self.trigger = True

    def process_and_publish(self):
        # TODO add check timestamps for dropped frames
        H, W, _ = self.data[0]['rgb'].shape
        n = len(self.data)
        dtype = [
            ('timestamp', 'float'),
            ('rgb', 'uint8', (H, W, 3)),
        ]
        images = np.zeros((n,), dtype=dtype)
        for i, v in enumerate(self.data):
            images[i]['timestamp'] = v['timestamp']
            images[i]['rgb'][:] = v['rgb']
        
        det = LEDDetector(False, False, False, self.pub_debug)
        rgb0 = self.data[0]['rgb']
        #mask = np.ones(dtype='bool', shape=rgb0.shape)
        tic = time.time()
        result = det.detect_led(images, self.frequencies, self.cell_size, self.crop_rect_normalized)
        self.pub_detections.publish(result)

        toc = time.time()-tic
        tac = time.time()-self.tinit
        rospy.loginfo('[%s] Detection done. Processing Time: %.2f'%(self.node_name, toc))
        print('[%s] Total Time taken: %.2f'%(self.node_name, tac))

        if(self.continuous):
            self.trigger = True
            self.sub_cam = rospy.Subscriber("camera_node/image/compressed",CompressedImage, self.camera_callback)
    
    def send_state(self, msg):
        msg.state = self.node_state
        self.pub_debug.publish(msg) 

if __name__ == '__main__':
    rospy.init_node('LED_detector_node',anonymous=False)
    node = LEDDetectorNode()
    rospy.spin()

