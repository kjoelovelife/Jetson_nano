#!/usr/bin/env python
# -*- coding: utf-8 -*
import  os
import  sys
import  tty, termios

import roslib;
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

cmd = Twist()
pub = rospy.Publisher('/jetbot_motors/cmd_str',  String, queue_size=10)

def keyboardLoop():
    #init
    rospy.init_node('teleop', anonymous=True)
    # Set rospy to exectute a shutdown function when exiting       
    rate = rospy.Rate(rospy.get_param('~hz', 6))

    #show tips
    print "Reading from keyboard"
    print "Use WASD keys to control the robot"
    print "Press Caps to move faster"
    print "Press q to quit"


    while not rospy.is_shutdown():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try :
            tty.setraw( fd )
            ch = sys.stdin.read( 1 )
        finally :
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if ch == 'w':
            rospy.loginfo("forward")
            cmd="forward"
        elif ch == 's':
            rospy.loginfo("backward")
            cmd="backward"
        elif ch == 'a':
            rospy.loginfo("left")
            cmd="left"
        elif ch == 'd':
            rospy.loginfo("right")
            cmd="right"
        elif ch == 'W':
            rospy.loginfo("forward")
            cmd="forward"
        elif ch == 'S':
            rospy.loginfo("backward")
            cmd="backward"
        elif ch == 'A':
            rospy.loginfo("left")
            cmd="left"
        elif ch == 'D':
            rospy.loginfo("right")
            cmd="right"
        elif ch == 'q':
            exit()
        else:
            rospy.loginfo("stop")
            cmd="stop"
        pub.publish(cmd)   
        rate.sleep()

        pub.publish("stop")

if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
