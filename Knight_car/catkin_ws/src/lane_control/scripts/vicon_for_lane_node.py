#!/usr/bin/env python
import rospy
import numpy as np
import tf
import math
from duckietown_msgs.msg import LanePose
from geometry_msgs.msg import PoseStamped

class ViconToLanePose(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.vicon_name = self.setupParameter("~vicon_veh_name","duckiecar")
        self.vicon_pose_queue = []
        self.last_pub_time = rospy.Time.now()

        self.lane_pose = None
        self.lane_pose_queue = None

        # Setup Parameters
        self.pub_freq = self.setupParameter("~pub_freq",50.0)  # 50 Hz
        self.pub_delay = self.setupParameter("~pub_delay",0.0)
        self.x_goal = self.setupParameter("~x_goal",0.0)
        self.phi_goal = self.setupParameter("~phi_goal",-math.pi/2)

        # Publications
        self.pub_lane_reading = rospy.Publisher("~lane_pose", LanePose, queue_size=1)
        # Subscriptions
        self.sub_vicon = rospy.Subscriber("~vicon_pose", PoseStamped, self.cbPose)
        
        # timer
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(1.0/self.pub_freq),self.cbTimerPub)
        self.timer_param = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamUpdate)

        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbParamUpdate(self,event):
        self.pub_freq = rospy.get_param("~pub_freq")
        self.pub_delay = rospy.get_param("~pub_delay")
        self.x_goal = rospy.get_param("~x_goal")
        self.phi_goal = rospy.get_param("~phi_goal")

    def cbPose(self,vicon_pose_msg):
        vicon_pose_msg.header.stamp = rospy.Time.now()
        self.vicon_pose_queue.append(vicon_pose_msg)

    def toLanePose(self,vicon_pose_msg):
        quat = vicon_pose_msg.pose.orientation
        euler_angles = tf.transformations.euler_from_quaternion([quat.x, quat.y,quat.z,quat.w])
        lane_pose = LanePose()
        lane_pose.d = vicon_pose_msg.pose.position.x - self.x_goal
        lane_pose.phi = euler_angles[2] - self.phi_goal
        lane_pose.header.stamp = vicon_pose_msg.header.stamp
        return lane_pose

    def cbTimerPub(self,event):
        if len(self.vicon_pose_queue) == 0:
            rospy.loginfo("[%s] No vicon readings yet." %(self.node_name))
            return

        if self.pub_delay <= 0.0:
            # Publisher the latest vicon pose directly
            vicon_pose = self.vicon_pose_queue.pop()
            lane_pose = self.toLanePose(vicon_pose)
            self.pub_lane_reading.publish(lane_pose)
        else:
            rospy.loginfo("[%s] len(vicon_pose_queue). %s" %(self.node_name,len(self.vicon_pose_queue)))
            while len(self.vicon_pose_queue) > 0:
                vicon_pose = self.vicon_pose_queue.pop(0)
                delaied_time = (rospy.Time.now() - vicon_pose.header.stamp).to_sec()
                if delaied_time < self.pub_delay:
                    # The first one within delay threshold
                    rospy.loginfo("[%s] Delaied time. %s" %(self.node_name,delaied_time))
                    lane_pose = self.toLanePose(vicon_pose)
                    self.pub_lane_reading.publish(lane_pose)
                    break
    
if __name__ == "__main__":
    rospy.init_node("vicon_for_lane_node", anonymous=False)
    node = ViconToLanePose()
    rospy.spin()
