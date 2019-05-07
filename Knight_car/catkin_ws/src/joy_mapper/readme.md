# joy_mapper package


# to test run:
1) connect joystick
2) roslaunch launch/joy_mapper_test.launch
3) the robot should move when you push buttons


## Dependencies
* `rospy`
* `sensor_msgs`: for the Joy.msg
* `duckietown_msgs`: for the CarControl.msg

# Node: joy_mapper.py
This node takes a sensor_msgs/Joy.msg and convert it to a duckietown_msgs/CarControl.msg. Publishes at fix interval with a zero order hold.

## Parameters
* `~pub_timestep`:
    Time steps (in seconds) between publishings of CarControl msgs. Default to 0.02 (50hz)

## Subscribe Topics
* `joy`: sensor_msgs/Joy.msg
    The Joy.msg from a `joy_node` of the `joy` package. The Vertical axis of the left stick maps to speed. The Horizontal axis of the right stick maps to steering.

## Publish Topics
* `~joy_control`: duckietown_msgs/CarControl.msg
    CarControl.speed: [-1,1] Positive is forward speed.
    CarControl.steering: [-1,1] Positive is steering left.
    CarControl.need_steering: default to false.

## Services
None

