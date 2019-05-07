#!/usr/bin/env python
import rospy
# from pkg_name.modulename import ModuleName
from duckietown_msgs.msg import CarControl
from dagu_car.daguddrive import DAGU_Differential_Drive
# import time

class DaguCar(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.car_like_mode = self.setupParam("~car_like_mode",True)

        # Setup publishers
        self.dagu = DAGU_Differential_Drive(car_like_mode=self.car_like_mode)
        self.dagu.setSpeed(0.0)
        self.dagu.setSteerAngle(0.0)

        self.control_msg = None
        # self.control_msg = CarControl()
        # self.control_msg.speed = 0.0
        # self.control_msg.steering = 0.0

        # Setup subscribers
        self.sub_topic = rospy.Subscriber("~car_control", CarControl, self.cbControl)

        # Create a timer that calls the cbTimer function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.02),self.cbTimer)

        self.last_cmd = None

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbControl(self,msg):
        self.control_msg = msg

    def cbTimer(self,event):
        if self.control_msg is None:
            return

        speed = self.control_msg.speed
        steer = self.control_msg.steering
        values = (speed, steer)
        
        # do not publish redundant values
        if self.last_cmd is not None:
            if values == (self.last_cmd.speed,self.last_cmd.steering):
                delta = event.current_real - self.last_cmd.header.stamp
                # unless 2 seconds elapsed
                if delta.to_sec() < 2.0:
                    return

        self.dagu.setSpeed(speed)
        self.dagu.setSteerAngle(steer)
        self.last_cmd = self.control_msg

    def on_shutdown(self):
        self.dagu.setSpeed(0.0)
        self.dagu.setSteerAngle(0.0)
        rospy.loginfo("[DaguCar] Shutting down.")

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('dagu_car', anonymous=False)
    
    # Create the DaguCar object
    node = DaguCar()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
