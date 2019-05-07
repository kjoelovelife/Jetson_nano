#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import Twist2DStamped, Pose2DStamped
from numpy import *


# Velocity to Position Node
# Authors: Robert Katzschmann
# Inputs: velocity
# Outputs: pose

class VelocityToPoseNode(object):
    def __init__(self):
        # Get node name and vehicle name
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]

        # Keep track of the last known pose
        self.last_pose = Pose2DStamped()
        self.last_theta_dot = 0
        self.last_v = 0
        
        # Setup the publisher and subscriber
        self.sub_velocity = rospy.Subscriber("~velocity", Twist2DStamped, self.velocity_callback, queue_size=1)
        self.pub_pose = rospy.Publisher("~pose", Pose2DStamped, queue_size=1)
        rospy.loginfo("[%s] Initialized.", self.node_name)

    def velocity_callback(self, msg_velocity):
        if self.last_pose.header.stamp.to_sec() > 0:  # skip first frame
            delta_t = (msg_velocity.header.stamp - self.last_pose.header.stamp).to_sec()
            [theta_delta, x_delta, y_delta] = self.integrate(self.last_theta_dot, self.last_v, delta_t)
            [theta_res, x_res, y_res] = self.propagate(self.last_pose.theta, self.last_pose.x, self.last_pose.y, theta_delta, x_delta, y_delta)

            self.last_pose.theta = theta_res
            self.last_pose.x = x_res
            self.last_pose.y = y_res

            # Stuff the new pose into a message and publish
            msg_pose = Pose2DStamped()
            msg_pose.header = msg_velocity.header
            msg_pose.header.frame_id = self.veh_name
            msg_pose.theta = theta_res
            msg_pose.x = x_res
            msg_pose.y = y_res
            self.pub_pose.publish(msg_pose)

        self.last_pose.header.stamp = msg_velocity.header.stamp
        self.last_theta_dot = msg_velocity.omega
        self.last_v = msg_velocity.v

    @staticmethod
    def integrate(theta_dot, v, dt):
        theta_delta = theta_dot * dt
        # to ensure no division by zero for radius calculation:
        if abs(theta_dot) < 0.000001:
            # straight line
            x_delta = v * dt
            y_delta = 0
        else:
            # arc of circle
            radius = v / theta_dot
            x_delta = radius * sin(theta_delta)
            y_delta = radius * (1.0 - cos(theta_delta))
        return [theta_delta, x_delta, y_delta]

    @staticmethod
    def propagate(theta, x, y, theta_delta, x_delta, y_delta):
        theta_res = theta + theta_delta
        x_res = x + x_delta * cos(theta) - y_delta * sin(theta)
        y_res = y + y_delta * cos(theta) + x_delta * sin(theta)
        return [theta_res, x_res, y_res]

if __name__ == '__main__':
    rospy.init_node('velocity_to_pose_node', anonymous=False)
    position_filter_node = VelocityToPoseNode()
    rospy.spin()
