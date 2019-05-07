#!/usr/bin/env python
import rospy
import time
import numpy as np
from Adafruit_LSM303 import Adafruit_LSM303
from Gyro_L3GD20 import Gyro_L3GD20
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField



class AdafruitIMU(object):

    # Physical constants
    G = 9.80665 # Standart gravity at sea level (should be g, but
                # capitalization rules due to coding practices)
    DEG_TO_RAD = 0.0174533 # degrees to radians

    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        # Setup compass and accelerometer        
        self.compass_accel = Adafruit_LSM303()

        # Setup gyroscope
        self.gyro = Gyro_L3GD20()

        # Setup Parameters
        self.pub_timestep = self.setupParam("~pub_timestep", 0.02)

        # Publications
        self.pub_imu = rospy.Publisher("~adafruit_imu", Imu, queue_size=10)
        self.pub_mag = rospy.Publisher("~adafruit_mag", MagneticField, queue_size=10)

        # timer
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.publish)

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparancy
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def publish(self, event):
        compass_accel = self.compass_accel.read()
        compass = compass_accel[0:3]
        accel = compass_accel[3:6]
        gyro = self.gyro.read()
        
        # Put together an IMU message
	imu_msg = Imu()
	imu_msg.header.stamp = rospy.Time.now()
        imu_msg.orientation_covariance[0] = -1
        imu_msg.angular_velocity = gyro[0] * DEG_TO_RAD
        imu_msg.angular_velocity = gyro[1] * DEG_TO_RAD
        imu_msg.angular_velocity = gyro[2] * DEG_TO_RAD
	imu_msg.linear_acceleration.x = accel[0] * G
	imu_msg.linear_acceleration.y = accel[1] * G
	imu_msg.linear_acceleration.z = accel[2] * G
    
	self.pub_imu.publish(imu_msg)

        # Put together a magnetometer message
	mag_msg = MagneticField()
	mag_msg.header.stamp = rospy.Time.now()
        mag_msg.magnetic_field.x = compass[0]
        mag_msg.magnetic_field.y = compass[1]
        mag_msg.magnetic_field.z = compass[2]
    
	self.pub_mag.publish(mag_msg)

if __name__ == "__main__":
    rospy.init_node("Adafruit_IMU", anonymous=False)
    adafruit_IMU = AdafruitIMU()
    rospy.spin()
