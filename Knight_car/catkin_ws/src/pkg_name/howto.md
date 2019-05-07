# How to write a ROS node in python for Duckietown
This document outline the process of writing a ROS pkg and nodes in python. It is recommend that you duplicate the `pkg_name` folder and edit the content of the files to make your own package.

# Package related Files

## CMakeLists.txt
Yes, you still need to have a `CMakeLists.txt` file even if you are just using python code in your package. But don't worry, it's pretty straight-forward. For a simple pkg, you only have to pay attention to the following parts.

At line 2 `project(pkg_name)`, this defines the name of the project.

Line 7 to 10, the `find_package`. You will have to specify the packages on which your pkg is dependent. In duckietown, most pkg should depend on `duckietown_msgs` to make use of the customized messages.

Line 21, `catkine_python_setup()` tells catkin to setup python related stuff for this pkg.

## package.xml
This files defines the meta data of the pkg. catkin makes use of it to flush out the dependency tree and figures out the order of compiling. Pay attention to the following parts.

`<name>pkg_name</name>` defines the name of the pkg. It has to match the project name in `CMakeLists.txt`.

`<description>` describes the pkg concisely.

`<maintainer` provides information of the maintainer.

`<build_depend>` and `<run_depend>`. The catkin packages this pkg depends on. This usually match the `find_package` in `CMakeLists.txt`.

## setup.py
This configures the python modules in this pkg.

The part to pay attention to is
```python
setup_args = generate_distutils_setup(
    packages=['pkg_name'],
    package_dir={'': 'include'},
)
```

The `packages`. is set to a list of strings of the name of the folders inside the `include` folder. The convention is to set the folder name the same as the pkg name. Here it's the `include/pkg_name` folder. You should put ROS-independent and/or reusable module (for other pkgs) in the `include/pkg_name` folder. Python files under this folder (for example, the `util.py`) will be available to scripts in the catkin workspace (this pkg and other pkg too) through
```python
from pkg_name.util import *
```

# Writing a node
Let's look at `src/talker.py` as an example. ROS nodes are put under the `src` folder and they have to be made executable to function properly. You can do so by`chmod +x talker.py`.

## Header
```python
#!/usr/bin/env python
import rospy
from pkg_name.util import HelloGoodbye #Imports module. Not limited to modules in this pkg. 
from std_msgs.msg import String #Imports msg
```

`#!/usr/bin/env python`, this specify that the script is written in python. Every ROS node in python should start with this line (or else it won't work properly.)

`import rospy` imports the rospy module necessary for all ROS nodes in python.

`from pkg_name.util import HelloGoodbye` imports HelloGoodby defined in the file `pkg_name/include/pkg_name/util.py`. Note that you can also include modules provided by other pkgs giving that you specify dependency in `CMakeLists.txt` and `package.xml`.

`from std_msgs.msg import String` imports the `String` msg defined in the `std_msgs` pkg. Note that you can use `rosmsg show std_msgs/String `
in a terminal to lookup the defintion of `String.msg`.

## Main
```python
if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('talker', anonymous=False)

    # Create the NodeName object
    node = Talker()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
```

`rospy.init_node('talker', anonymous=False)` initialize a node named `talker`. Note that this name can be overwritten by a launch file. The launch file can also push this node down namespaces. If the `anonymous` argument is set to `True` then a random string of numbers will be append to the name of the node. Usually we don't use anonymous nodes.

`node = Talker()` creates an instance of the Talker object. More details in the next section.

`rospy.on_shutdown(node.on_shutdown)` ensures that the `node.on_shutdown` will be called when the node is shutdown.

`rospy.spin()` blocks to keep the script alive. This makes sure the node stays alive and all the publication/subscriptions work correctly.

## Talker
```python
class Talker(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
        # Setup subscriber
        self.sub_topic_b = rospy.Subscriber("~topic_b", String, self.cbTopic)
        # Read parameters
        self.pub_timestep = setupParameter("~pub_timestep",1.0)
        # Create a timer that calls the cbTimer function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbTopic(self,msg):
        rospy.loginfo("[%s] %s" %(self.node_name,msg.data))

    def cbTimer(self,event):
        singer = HelloGoodbye()
        # Simulate hearing something
        msg = String()
        msg.data = singer.sing("duckietown")
        self.pub_topic_name.publish(msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))
```
### constructor
```python
self.node_name = rospy.get_name()
```
saves the name of the node. Including the name of the node in printouts makes them more informative.

```python
rospy.loginfo("[%s] Initialzing." %(self.node_name))
```
prints to ROS info.

```python
self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
```
defines a publisher which publishes a `Sting` msg to the topic `~topic_a`. Note that the `~` in the name of topic under the namespace of the node. More specifically, this will actually publishes to `talker/topic_a` instead of just `topic_a`. The `queue_size` is usually set to 1 on all publishers. For more details see [rospy overview: publisher and subscribers](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers)

```python
self.sub_topic_b = rospy.Subscriber("~topic_b", String, self.cbTopic)
```
defines a subscriber which expects a `String` message and subscribes to `~topic_b`. The message will be handled by the `self.cbTopic` callback function. Note that similar to the publisher, the `~` in the topic name puts the topic under the namespace of the node. In this case the subscriber actually subscribes to the topic `talker/topic_b`.

It is strongly encouraged that a node always publishers and subscribes to topics under their `node_name` namespace. In other words, always put a `~` in front of the topic names when you defines a publisher or a subscriber. They can be easily remapped in a launch file. This makes the node more modular and minimizes the possibility of confusion and naming conflicts. See the launch file section for how remapping works.

```python
self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
```
Sets the value of self.pub_timestep to the value of the parameter `~pub_timestep`. If the parameter doesn't exist (not set in the launch file), then set it to the default value `1.0`. The `setupParameter` function also writes the final value to the parameter server. This means that you can `rosparam list` in a terminal to check the actual values of parameters being set. 

```python
self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)
```
defines a timer that calls the `self.cbTimer` function every `self.pub_timestep` seconds.

### Timer callback
```python
def cbTimer(self,event):
    singer = HelloGoodbye()
    # Simulate hearing something
    msg = String()
    msg.data = singer.sing("duckietown")
    self.pub_topic_name.publish(msg)
```
Everytime the timer ticks, a message is generated and published.

### Subscriber callback
```python
def cbTopic(self,msg):
    rospy.loginfo("[%s] %s" %(self.node_name,msg.data))
```
Everytime a message is published to `~topic_b`, the `cbTopic` function is called. It simply prints the msg using `rospy.loginfo`.

# Launch File
You should always write a launch file to launch a node. It also serves as a documentation on the IOs of the node. Let's take a look at `launch/test.launch`.
```
<launch>
    <node name="talker" pkg="pkg_name" type="talker.py" output="screen">
        <!-- Setup parameters -->
        <param name="~pub_timestep" value="0.5"/>
        <!-- Remapping topics -->
        <remap from="~topic_b" to="~topic_a"/>
    </node>
</launch>
```
For the `<node>`, the `name` specify the name of the node, which overwrites `rospy.init_node()` in the `__main__` of `talker.py`. The `pkg` and `type` specify the pkg and the script of the node, in this case it's `talke.py`. Don't forget the .py in the end (and remember to make the file executable through chmod). The `output="screen"` direct all the rospy.loginfo to the screen, without this you won't see any printouts (useful when you want to suppress a node that's too talkative.)

The `<param>` can be used to set the parameters. Here we set the `~pub_timestep` to `0.5`. Note that in this case this sets the value of `talker/pub_timestep` to `0.5`.

The `<remap>` is used to remap the topic names. In this case we are replacing `~topic_b` with `~topic_a` so that the subscriber of the node actually listens to its own publisher. Replace the line with
```
<remap from="~topic_b" to="talker/topic_a"/>
```
will have the same effect. This is redundant in this case but very useful when you want to subscribe to a topic published by another node.

# Testing the node
First of all, you have to `catkin_make` the pkg even if it only uses python. `catkin` makes sure that the modules in the include folder and the messages are available to the whole workspace. You can do so by
```bash
$ cd ~/duckietown/catkin_ws
$ catkin_make
```

Ask ROS to reindex the packages so that you can auto-complete most things.
```bash
$ rospack profile
```

Now you can launch the node by the launch file.
```bash
$ roslaunch pkg_name test.launch
```
You should see something like this in the terminal
```
... logging to /home/shihyuan/.ros/log/d4db7c80-b272-11e5-8800-5c514fb7f0ed/roslaunch-Wolverine-15961.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://Wolverine.local:33925/

SUMMARY
========

PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.16
 * /talker/pub_timestep: 0.5

NODES
  /
    talker (pkg_name/talker.py)

auto-starting new master
process[master]: started with pid [15973]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to d4db7c80-b272-11e5-8800-5c514fb7f0ed
process[rosout-1]: started with pid [15986]
started core service [/rosout]
process[talker-2]: started with pid [15993]
[INFO] [WallTime: 1451864197.775356] [/talker] Initialzing.
[INFO] [WallTime: 1451864197.780158] [/talker] ~pub_timestep = 0.5 
[INFO] [WallTime: 1451864197.780616] [/talker] Initialzed.
[INFO] [WallTime: 1451864198.281477] [/talker] Goodbye, duckietown.
[INFO] [WallTime: 1451864198.781445] [/talker] Hello, duckietown.
[INFO] [WallTime: 1451864199.281871] [/talker] Goodbye, duckietown.
[INFO] [WallTime: 1451864199.781486] [/talker] Hello, duckietown.
[INFO] [WallTime: 1451864200.281545] [/talker] Goodbye, duckietown.
[INFO] [WallTime: 1451864200.781453] [/talker] Goodbye, duckietown.
```

Open another terminal and
```
$ rostopic list
```
You should see
```
/rosout
/rosout_agg
/talker/topic_a
```

In the same terminal
```
$ rosparam list
```
You should see
`/talker/pub_timestep`

You can see the parameters and the values of the `talker` node with
```
$ rosparam get /talker
```

# Documentation
You should document the parameters and the publish/subscribe topic names of each node in your package. The user should not have to look at the source code to figure out how to use the nodes.

# Guidelines
* Make sure to put all topics (publish or subscribe) and parameters under the namespace of the node with `~`. This makes sure that the IO of the node is crystal clear.
* Always include the name of the node in the printouts.
* Always provide a launch file that includes all the parameters (using `<param>`) and topics (using `<remap>`) with each node.