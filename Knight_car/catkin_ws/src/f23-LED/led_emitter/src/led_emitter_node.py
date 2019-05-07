#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import Float32, Int8, String
from rgb_led import RGB_LED
from duckietown_msgs.msg import BoolStamped


class LEDEmitter(object):
    def __init__(self):
        self.led = RGB_LED()
        self.node_name = rospy.get_name()
        self.pub_state = rospy.Publisher("~current_led_state",Float32,queue_size=1)
        self.sub_pattern = rospy.Subscriber("~change_color_pattern", String, self.changePattern)
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped,self.cbSwitch)
        self.cycle = None
        
        self.is_on = False
        self.active = True

        self.protocol = rospy.get_param("~LED_protocol") #should be a list of tuples

        self.pattern_off = [[0,0,0]] * 5

        scale = 0.5
        for _, c in self.protocol['colors'].items():
            for i in range(3):
                c[i] = c[i]  * scale

        self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(.1), self.cycleTimer)
        self.current_pattern_name = None
        self.changePattern_('CAR_SIGNAL_A')

    def cbSwitch(self, switch_msg): # active/inactive switch from FSM
        self.active = switch_msg.data


    def cycleTimer(self,event):
        if not self.active:
            return
        if self.is_on:
            for i in range(5):
                self.led.setRGB(i, [0, 0, 0])
                self.is_on = False
        else:
            for i in range(5):
                self.led.setRGB(i, self.pattern[i])
                self.is_on = True

    def changePattern(self, msg):
        self.changePattern_(msg.data)

    def changePattern_(self, pattern_name):
        if pattern_name:
            if (self.current_pattern_name == pattern_name):
                return
            else:
                self.current_pattern_name = pattern_name

            rospy.loginfo('changePattern(%r)' % pattern_name)
            color = self.protocol['signals'][pattern_name]['color']
            self.cycle = self.protocol['signals'][pattern_name]['frequency']
            print("color: %s, freq (Hz): %s "%(color, self.cycle))

            self.pattern = [[0,0,0]] * 5
            self.pattern[2] = self.protocol['colors'][color]
            #print(self.pattern)

            if pattern_name in ['traffic_light_go', 'traffic_light_stop']:
                self.pattern = [self.protocol['colors'][color]] * 5

            self.changeFrequency()

    def changeFrequency(self): 
        try:
            #self.cycle = msg.data
            self.cycle_timer.shutdown()
            #below, convert to hz
            d = 1.0/(2.0*self.cycle)
            self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(d), self.cycleTimer)
        except ValueError as e:
            self.cycle = None
            self.current_pattern_name = None
    	self.pub_state.publish(float(self.cycle))

if __name__ == '__main__':
    rospy.init_node('led_emitter',anonymous=False)
    node = LEDEmitter()
    rospy.spin()

