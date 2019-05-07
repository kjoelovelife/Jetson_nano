#!/usr/bin/env python
import rospy
import time
#from led_detection.LEDDetector import LEDDetector
from std_msgs.msg import Byte
from duckietown_msgs.msg import FSMState, Vector2D, AprilTags, LEDDetection, LEDDetectionArray, LEDDetectionDebugInfo, SignalsDetection 
from sensor_msgs.msg import CompressedImage
#from duckietown_utils.bag_logs import numpy_from_ros_compressed
#import numpy as np

#this is a stup for traffic light testing

class LEDInterpreterNode(object):
	def __init__(self):

		self.node = rospy.init_node('LED_interpreter_node',anonymous=True)
		self.node_name = rospy.get_name()
		
		self.trafficLightIntersection = False		

		rospy.loginfo('[%s] Intersection Type: %s'%(self.node_name, rospy.get_param("~intersectionType")))
		if rospy.get_param("~intersectionType") == "trafficLight":
			self.trafficLightIntersection = True


		self.protocol = rospy.get_param("~LED_protocol") #should be a list of tuples
		self.label = rospy.get_param("~location_config") # should be a list

		self.lightGo = self.protocol['signals']['traffic_light_go']['frequency']
		self.lightStop = self.protocol['signals']['traffic_light_stop']['frequency']
		self.carSignalA = self.protocol['signals']['CAR_SIGNAL_A']['frequency']
		self.carSignalB = self.protocol['signals']['CAR_SIGNAL_B']['frequency']
		self.carSignalC = self.protocol['signals']['CAR_SIGNAL_C']['frequency']

		self.signalFrequencies = [self.carSignalA, self.carSignalB,self.carSignalC]
		self.vehicleSignals = [SignalsDetection.SIGNAL_A,SignalsDetection.SIGNAL_B,SignalsDetection.SIGNAL_C] 


		#initialize the standard output message
		self.front = SignalsDetection.NO_CAR
		self.right = SignalsDetection.NO_CAR
		self.left = SignalsDetection.NO_CAR

		self.traffic_light_state = SignalsDetection.NO_TRAFFIC_LIGHT

		#publishers and subscribers
		self.pub_interpret = rospy.Publisher("~signals_detection", SignalsDetection, queue_size = 1)
		#self.sub_tags = rospy.Subscriber("apriltags_postprocessing_fast_node/apriltags", AprilTags, self.CheckTags)
		self.sub_LEDs = rospy.Subscriber("~raw_led_detection", LEDDetectionArray, self.Interpreter, queue_size = 1)
		#self.switch = rospy.Subscriber("~mode", FSMState, self.seeSwitch)
		rospy.loginfo("Initialized.")

		#while not rospy.is_shutdown():
            	#	self.publish_topics()
            	#	rospy.sleep(0.1)


	#def CheckTags(self, msg):
	#task of this is to check on what type of intersection we are
	#	if self.active:
	#		if not self.setIntersectionType:
	#			for info in msg.infos:
	#				if info.traffic_sign_type == info.STOP:
	#					self.trafficLightIntersection = False
	#				break
	#			self.setIntersectionType = True

	#def seeSwitch(self, msg):
	#	if msg.state == "COORDINATION":
	#		rospy.loginfo("coordination mode active")
	#		self.active = True
	#	else: #reset parameters
	#		rospy.loginfo("coordination mode inactive")
	#		self.active = False
	#		self.setIntersectionType = False
	#		self.hasObservedSignals = False
	#		self.trafficLightIntersection = True


	def Interpreter(self, msg):
		rospy.loginfo("[%s] Read a message from Detector" %(self.node_name))
		#initialize the standard output message
		self.front = SignalsDetection.NO_CAR
		self.right = SignalsDetection.NO_CAR
		self.left = SignalsDetection.NO_CAR

		self.traffic_light_state = SignalsDetection.NO_TRAFFIC_LIGHT

		#case with a traffic light
		if self.trafficLightIntersection:
			for item in msg.detections:
				rospy.loginfo("[%s]:\n pixel = %f\n top bound = %f\n measured frequence=%f\n go frequency =%f" %(self.node_name, item.pixels_normalized.y,self.label['top'],item.frequency,self.lightGo))
				if item.pixels_normalized.y < self.label['top']:
					if abs(item.frequency - self.lightGo) < 0.1:
						self.traffic_light_state = SignalsDetection.GO
						break
					else:
						self.traffic_light_state = SignalsDetection.STOP
						break

		#case with stop sign intersection	
		else:
			for item in msg.detections:
				rospy.loginfo("[%s]:\n pixel = %f\n right bound = %f\n left bound =%f\n measured frequence=%f\n" %(self.node_name, item.pixels_normalized.x,self.label['right'],self.label['left'],item.frequency))
				#check if front vehicle detection
				if item.pixels_normalized.x < self.label['right'] and item.pixels_normalized.y > self.label['top']:
					#check signal of that vehicle
					detected_freq = item.frequency
					for i in range(len(self.signalFrequencies)):
						if abs(self.signalFrequencies[i] - detected_freq) < 0.1:
							self.front = self.vehicleSignals[i]
							break

				#check if right vehicle detection
				if item.pixels_normalized.x > self.label['right'] and item.pixels_normalized.y > self.label['top']:
					#check signal of that vehicle
					detected_freq = item.frequency
					for i in range(len(self.signalFrequencies)):
						if abs(self.signalFrequencies[i] - detected_freq) < 0.1:
							self.right = self.vehicleSignals[i]
							break	
	
		rospy.loginfo("[%s] The observed LEDs are:\n Front = %s\n Right = %s\n Traffic light state = %s" %(self.node_name, self.front, self.right,self.traffic_light_state))
		self.pub_interpret.publish(SignalsDetection(front=self.front,right=self.right,left=self.left,traffic_light_state=self.traffic_light_state))
					
					

	#def publish_topics(self):
	#	if self.active:
	#		if self.hasObservedSignals:
	#			self.pub_interpret.publish(SignalsDetection(front=self.front,right=self.right,left=self.left,traffic_light_state=self.traffic_light_state))	



	# def setupParam(self,param_name,default_value):
	# 		value = rospy.get_param(param_name,default_value)
	# 		rospy.set_param(param_name,value) #Write to parameter server for transparancy
	# 		# rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
	# 		return value


	def onShutdown(self):
		    rospy.loginfo("[LED Interpreter] Shutdown.")

if __name__ == '__main__':
   # rospy.init_node('virtual_mirror_qlai_tester',anonymous=False)
    interpreternode = LEDInterpreterNode()
    rospy.on_shutdown(interpreternode.onShutdown)
    rospy.spin()

