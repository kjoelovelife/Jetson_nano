#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import CarControl

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('dagu_car_tester', anonymous=False)
    pub = rospy.Publisher("~car_control",CarControl,queue_size=1)
    rospy.loginfo("[dagu_car_tester] Initializing.")
    rospy.sleep(1.0)
    # Full speed ahead
    pub.publish(CarControl(speed=1.0,steering=0.0))
    rospy.loginfo("[dagu_car_tester] Full speed ahead.")
    rospy.sleep(3.0)
    # Full speed reverse
    pub.publish(CarControl(speed=-1.0,steering=0.0))
    rospy.loginfo("[dagu_car_tester] Full speed reverse")
    rospy.sleep(3.0)
    # Full speed left turn
    pub.publish(CarControl(speed=1.0,steering=1.0))
    rospy.loginfo("[dagu_car_tester] Full speed left turn")
    rospy.sleep(3.0)
    # Full speed left turn
    pub.publish(CarControl(speed=1.0,steering=-1.0))
    rospy.loginfo("[dagu_car_tester] Full speed right turn")
    rospy.sleep(3.0)
    # Stop
    pub.publish(CarControl(speed=0.0,steering=0.0))
    rospy.loginfo("[dagu_car_tester] Stop")
    rospy.sleep(3.0)

    rospy.loginfo("[dagu_car_tester] Testing Done")
    # rospy.spin()