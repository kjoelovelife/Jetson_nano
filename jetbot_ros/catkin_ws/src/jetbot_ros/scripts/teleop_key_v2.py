#!/usr/bin/env python
# -*- coding: utf-8 -*

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import sys, select, termios, tty

import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

START_LIN_VALUE = 0.6
START_ANG_VALUE = 0.4

MAX_LIN_VEL = 1.0 - START_LIN_VALUE
MAX_ANG_VEL = 1.0 - START_ANG_VALUE # default : 13.0 

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.01
    


msg = """
Control Your Jetbot!!
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease linear velocity (Max : 1.0)
a/d : increase/decrease angular velocity (Max : 1.0)
space key, s : force stop
CTRL-C to quit
"""
error = """
Communications Failed
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    global START_LIN_VALUE , START_ANG_VALUE

    if target_linear_vel > 0 :
        target_linear_vel = target_linear_vel + START_LIN_VALUE
    elif target_linear_vel < 0 :
        target_linear_vel = target_linear_vel - START_LIN_VALUE
    else :
        target_linear_vel = 0

    if target_angular_vel > 0 :
        target_angular_vel = target_angular_vel + START_ANG_VALUE
    elif target_angular_vel < 0 :
        target_angular_vel = target_angular_vel - START_ANG_VALUE
    else :
        target_angular_vel = 0

    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)
    return vel

def AddStartLinValue(vel):
    global START_LIN_VALUE
    if vel > 0 :
        vel = vel + START_LIN_VALUE
    elif vel < 0 :
        vel = vel - START_LIN_VALUE
    else :
        vel = 0
    return vel

def AddStartAngValue(vel):
    global START_ANG_VALUE
    if vel > 0 :
        vel = vel + START_ANG_VALUE
    elif vel < 0 :
        vel = vel - START_ANG_VALUE
    else :
        vel = 0
    return vel

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop', anonymous=False)
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(rospy.get_param('~hz', 6))

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0


    try:
        print msg
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print vels(target_linear_vel, target_angular_vel)
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print msg
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = AddStartLinValue(control_linear_vel) ; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = AddStartAngValue(control_angular_vel)

            pub.publish(twist)

    except:
        print error

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
