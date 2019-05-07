#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState
from std_msgs.msg import String

class LEDPatternSwitchNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        # rospy.loginfo("[%s] Initializing " %(self.node_name))
        # Read parameters
        self.mappings = rospy.get_param("~mappings")
        source_topic_dict = rospy.get_param("~source_topics")
        self.current_src_name = "joystick" # by default if fsm is missing

        # Construct publisher
        self.pub_cmd = rospy.Publisher("~change_color_pattern",String,queue_size=1)
        
        # Construct subscribers
        self.sub_fsm_state = rospy.Subscriber(rospy.get_param("~mode_topic"),FSMState,self.cbFSMState)

        self.sub_dict = dict()
        for src_name, topic_name in source_topic_dict.items():
            self.sub_dict[src_name] = rospy.Subscriber(topic_name,String,self.msgincb,callback_args=src_name)

        rospy.loginfo("[%s] Initialized. " %(self.node_name))

    def cbFSMState(self,fsm_state_msg):
        self.current_src_name = self.mappings.get(fsm_state_msg.state)
        if self.current_src_name is None:
            rospy.logwarn("[%s] FSMState %s not handled. No msg pass through the switch." %(self.node_name,fsm_state_msg.state))
        else: 
            rospy.loginfo("[%s] Led pattern switched to %s in state %s." %(self.node_name,self.current_src_name,fsm_state_msg.state))

    def msgincb(self,msg,src_name):

        if src_name == self.current_src_name:
            #rospy.loginfo("[%s] %s callback matches, publishing"%(self.node_name,src_name))
            self.pub_cmd.publish(msg)
        #else:
            #rospy.loginfo("[%s] %s callback does not match, not publishing"%(self.node_name,src_name))

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('LED_pattern_switch_node', anonymous=False)
    # Create the DaguCar object
    node = LEDPatternSwitchNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
