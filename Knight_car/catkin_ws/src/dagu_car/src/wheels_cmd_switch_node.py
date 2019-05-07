#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, FSMState
class WheelsCmdSwitchNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
<<<<<<< HEAD
        
        self.mode_msg = FSMState()
        self.mode_msg.state = FSMState.LANE_FOLLOWING
        self.cmd_dict = {}
        self.cmd_dict[FSMState.LANE_FOLLOWING] = None
        self.cmd_dict[FSMState.INTERSECTION_CONTROL] = None
        self.cmd_dict[FSMState.COORDINATION] = None
        self.cmd_dict[FSMState.VEHICLE_AVOIDANCE] = None
        self.cmd_dict[FSMState.OBSTACLE_AVOID] = None

        self.mode_name_dict = {}
        self.mode_name_dict[FSMState.LANE_FOLLOWING] = "LANE_FOLLOWING"
        self.mode_name_dict[FSMState.INTERSECTION_CONTROL] = "INTERSECTION_CONTROL"
        self.mode_name_dict[FSMState.COORDINATION] = "COORDINATION_CONTROL"
        self.mode_name_dict[FSMState.VEHICLE_AVOIDANCE] = "VEHICLE_AVOIDANCE"
        self.mode_name_dict[FSMState.OBSTACLE_AVOID] = "OBSTACLE_AVOID"

        # Setup publishers
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)

        # Setup subscribers
        self.sub_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        self.sub_lane = rospy.Subscriber("~wheels_cmd_lane", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1, callback_args=FSMState.LANE_FOLLOWING)
        self.sub_lane = rospy.Subscriber("~simple_stop_commands", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1, callback_args=FSMState.OBSTACLE_AVOID)
        self.sub_interestion = rospy.Subscriber("~wheels_cmd_intersection", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1, callback_args=FSMState.INTERSECTION_CONTROL)
        self.sub_coordination = rospy.Subscriber("~wheels_cmd_coordination", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1, callback_args=FSMState.COORDINATION)
        self.sub_vehicle_avoidance = rospy.Subscriber("~wheels_cmd_avoidance", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1, callback_args=FSMState.VEHICLE_AVOIDANCE)

    def pubWheelsCmd(self):
        cmd_msg = self.cmd_dict[self.mode_msg.state]
        if cmd_msg is None:
            cmd_msg = WheelsCmdStamped()
            cmd_msg.vel_left=0.0
            cmd_msg.vel_right=0.0
            cmd_msg.header.stamp = rospy.Time.now()
        self.pub_wheels_cmd.publish(cmd_msg)

    def cbMode(self,mode_msg):
        if not self.mode_msg.state == mode_msg.state:
            rospy.loginfo("[%s] Switching to %s" %(self.node_name,self.mode_name_dict[mode_msg.state]))
            #THESE LINES WERE NOT IN THE IF STATEMENT ORIGINALLY!!!!!
            self.mode_msg = mode_msg
            # Always publish a cmd when changing mode.
            self.pubWheelsCmd()

    def cbWheelsCmd(self,cmd_msg,cb_args):
        # Save the cmd_msg 
        self.cmd_dict[cb_args] = cmd_msg
        # Publish if the received cmd channel matches the current mode
        #rospy.loginfo("cb_args is [%d]. Mode is [%d]." %(cb_args, self.mode_msg.state))
        if cb_args == self.mode_msg.state:
            self.pubWheelsCmd()
=======
        # Read parameters
        self.mappings = rospy.get_param("~mappings")
        source_topic_dict = rospy.get_param("~source_topics")
        self.current_src_name = None

        # Construct publisher
        self.pub_cmd = rospy.Publisher("~wheels_cmd",WheelsCmdStamped,queue_size=1)
        
        # Construct subscribers
        self.sub_fsm_state = rospy.Subscriber(rospy.get_param("~mode_topic"),FSMState,self.cbFSMState)

        self.sub_dict = dict()
        for src_name, topic_name in source_topic_dict.items():
            self.sub_dict[src_name] = rospy.Subscriber(topic_name,WheelsCmdStamped,self.cbWheelsCmd,callback_args=src_name)

    def cbFSMState(self,fsm_state_msg):
        self.current_src_name = self.mappings.get(fsm_state_msg.state)
        if self.current_src_name is None:
            rospy.logwarn("[%s] FSMState %s not handled. No msg pass through the switch." %(self.node_name,fsm_state_msg.state))
>>>>>>> devel-fsm

    def cbWheelsCmd(self,msg,src_name):
        if src_name == self.current_src_name:
            self.pub_cmd.publish(msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('wheels_cmd_switch_node', anonymous=False)
    # Create the DaguCar object
    node = WheelsCmdSwitchNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
