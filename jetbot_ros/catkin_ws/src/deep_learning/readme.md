# Jetbot_ros - package for deep_learning
----
This package is for Deep_Learning, rewrite the code below to build it.
* reference: https://github.com/NVIDIA-AI-IOT/jetbot

# Developer
* [Wei-chih Lin](kjoelovelife@gmail.com)

# Software Version
* Operating System: [Nvidia Jetson_nano, ubuntu 18.04](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write)
* ROS(1.0):  melodic
* Opencv: 4.3.0
* Cuda: 10.2
* python: 2.7
* pytorch: 1.4.0

# Package struct

It contains three package:

* img_recognition
-- can use [ROS service](http://wiki.ros.org/Services) to collect image, then train model and inference model. If you can, design the Jetbot_ros motion after inference model.
* road_following
-- can use [ROS dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile) to adjist parameter for collecting image, then train model and inference model to recognitize line with color you want to follow.  
* self_driving
-- combin package img_recognition and road_following, let Jetbot_ros can run on the road with line and see 

# Getting started
----
Before using package self-driving, we need finished package img_recognition and road_following. In package img_recognition and road_following, we have three steps to do:
1. Data collection
2. training
3. inference model and development

# Use package img_recognition 
----
First, we use packae img_recognition to recognitize object, such as traffic light, stone, obstacle and more. You can think what will appear on the road. In this example, we'll distinguish "free" and "blocked" images.

## Step 1. Data collection

Before collecting data, we need to make folder to save image. Open Terminal and use the code below to make folder:

```sh
$ rosrun img_recognition mkdir.py -n "folder_name"
```

Note: In this example, we need replcae "folder_name" to "free" and "blocked". After do it, you can find two folders in [ ../catkin_ws/src/deep_learning/img_recognition/img ]. At the same time, you can check out the content in [ ../catkin_ws/src/deep_learning/img_recognition/param/image_label.yaml ], it will be

> blocked: 0

> free: 0

Great! If you check out the folder_name is the same as content in image_label.yaml, you finished make folder to save image. The number in the image_label.yaml is the image number.

Now, you can type the code below to start the launch file:

```sh
$ roslaunch img_recognition save_image.launch label:="folder_name" 
```

Note: the "folder_name" must be the same as one of you make before. In this example, will be the "free" or "blocked". Don't worry, you can change the "folder_name" while collecting data. 

Also, the interval between taking picture default is 0.5 seconds.If you want to change the interval, you can type the code when start the file "save_image.launch":

```sh
$ roslaunch img_recognition save_image.launch label:="folder_name" picture_interval:="time"
```

Note: unit of picture_interval is second.

if you start without error, will see the information:

> [INFO] [1613979930.707966]: [/Jetbot_ros01/save_image] ~label = free 

> [INFO] [1613979931.346135]: [/Jetbot_ros01/save_image] ~picture_interval = 0.5 

> [INFO] [1613979931.523790]: [/Jetbot_ros01/save_image] Your image label : free 

> [INFO] [1613979931.528219]: [/Jetbot_ros01/save_image] If your label is wrong, please change the label.

> [INFO] [1613979931.533456]: [/Jetbot_ros01/save_image] Remember checkout the image size(width=224 ,height=224).

> [INFO] [1613979931.541482]: [/Jetbot_ros01/save_image] You can use service with [srv_client_save_image.py] to start collecting your data!

> [INFO] [1613979931.549191]: [/Jetbot_ros01/save_image] The label(folder) and image you have :

> [INFO] [1613979931.552938]: [/Jetbot_ros01/save_image] [('blocked', 0), ('free', 0)]

Great! We can save image now! Please take your object want to recognitize in fron of the camera. And then, open another new terminal and type the code below:

```sh
$ rosservice call /[$hostname]/save_image/save_image_action -- true 
```

Note: [$hostname] is your hostname on Jetbot_ros, package img_recognition will auto-use the hostname to register the node name. In this example, the hostname is "rosk01", so you can see the [Info]... above is the [/Jetbot_ros01/save_image].. . 
If taking picture successfully, you'll see the information in terminal "save_image.launch":

>[INFO] [1613980950.029004]: [/Jetbot_ros01/save_image] save image in /home/icshop/Jetbot_ros/catkin_ws/src/deep_learning/img_recognition/image/free 

>[INFO] [1613980950.529054]: [/Jetbot_ros01/save_image] save image in /home/icshop/Jetbot_ros/catkin_ws/src/deep_learning/img_recognition/image/free

While taking picture, you should put the object in any angle and background, this will avoid the model be the "overfitting model".

if you want to stop taking picture, please type the code below in terminal can type code(not the terminal "save_image.launch"):

```sh
$ rosservice call /[$hostname]/save_image/save_image_action -- false 
```

Please repeate the same steps to save image in another folder.
When finished saving picture, please press [ctrl] + [c] in terminal "save_image.launch" to cancel the save_image.launch, and then you can see the content in [ ../catkin_ws/src/deep_learning/img_recognition/param/image_label.yaml ].

## Step2. Training

Befor training the model, we'll configure the parameter about the model. Please check out and modify the file  [ ../catkin_ws/src/deep_learning/img_recognition/param/train_model.yaml ].

Note: Now we just support the model struct is "alexnet". And please check out the save_model_name, will not be the same as any model in [ ../catkin_ws/src/deep_learning/img_recognition/model ]. If your model name is repeat, system will auto add time after your model name.  

When you finished modified parameter about training model, you can type the code below to train model:

```sh
$ roslaunch img_recognition train_model.launch
```

In this example, information is:

> [INFO] [1613987099.768488]: /Jetbot_ros01/train_model  Initializing train_model.py......

> [WARN] [1613987100.446557]: Model name repeat nad will be reset! Now the name is: best3-2021-02-22-17-45-00

> [INFO] [1613987100.654610]: You use model [alexnet]. Need some time to load model...

> ...

> [INFO] [1613987185.089697]: Epoch: 1, accuracy: 1.0, loss: 0.173195719719, time: 61.59.

> [INFO] [1613987206.301567]: Epoch: 2, accuracy: 1.0, loss: 0.0143429040909, time: 

> ...

> [INFO] [1613987332.226332]: Now you can press [ctrl] + [c] to shutdwon the lauch file.

> [Jetbot_ros01/train_model-2] process has died [pid 7250, exit code -11, cmd 
/home/icshop/Jetbot_ros/catkin_ws/src/deep_learning/img_recognition/src/train_model.py __name:=train_model __log:=/home/icshop/.ros/log/9d4f9b82-74f2-11eb-9604-a4c3f0eb8755/Jetbot_ros01-train_model-2.log].
log file: /home/icshop/.ros/log/9d4f9b82-74f2-11eb-9604-a4c3f0eb8755/Jetbot_ros01-train_model-2*.log

Great! Because the node will shutdown automatically, the last information will use "red text" to tell you there is an error occur. Dont't worry, if you can see the text "Now you can press [ctrl] + [c] to shutdwon the lauch file.", means training done and successfully. Just press [ctrl] + [c] to cancel the code in terminal "train_model.launch".

Now you can find the trained model in [ ../catkin_ws/src/deep_learning/img_recognition/model ], and know information for this trained model in [ ../catkin_ws/src/deep_learning/img_recognition/model/recording.yaml ].

## Step3. inference model and development

Before inferencing the model, we need to modify the param in [ ../catkin_ws/src/deep_learning/img_recognition/param/inference_model.yaml ]. 
Please check out the "model_pth", must be the same as one of the model in [ ../catkin_ws/src/deep_learning/img_recognition/model ].    

And then, you can type the code below to start inferencing the model:

```sh
$ roslaunch img_recognition inference.launch
```

In this example, when you see the information below, means Jetbot_ros can recognition the object you want!

> ...

> [INFO] [1613989152.729438]: Start to recognitize image! Theer are 2 object you can recognitize: ['blocked', 'free']

> [INFO] [1613989152.734968]: You can listen the topic to see how much the confidence about object: /Jetbot_ros01/inference_model/inference

> ...

You can listen the topic to see what object to recognitize by typing the code in another terminal:

```sh
$ rostopic echo /[$hostname]/inference_model/inference
```

Note: [$hostname] is your hostname on Jetbot_ros, package img_recognition will auto-use the hostname to register the node name. In this example, the hostname is "rosk01". And then you can see the information on terminal:

> labels: [free, blocked]

> confidence: [0.7282347083091736, 0.27176526188850403]

> ...

Awesome! we can use Jetbot_ros to recognitize object. When you want stop recognitizing, remember cancael the code using pressing [ctrl] + [c] in terminal "inference.launch".

# Use package road_followig
----
Except, instead of classification, you can use a different fundamental technique, regression, that we'll use to enable Jetbot_ros to follow a road (or really, any path or target point).

Place the Jetbot_ros in different positions on a path (offset from center, different angles, etc)
Remember from collision avoidance, data variation is key!

Display the live camera feed from the robot
Store the X, Y values of this green dot along with the image from the robot's camera    

## Step1. Data collection

We use dynamic configure to adjust parameter about (x, y). Please type your code in the new terminal bellow:

```sh
$ roslaunch road_following display_xy.launch
```

Great! Let's start to collect data about road. You can start "another computer" and running ROS across multiple machines then type the code below:

```sh
$ rosrun rqt_image_view rqt_image_view image:=/[$hostname]/display_xy/image/raw/draw_xy_line
```

Then you can see the image from camera. There will a line on the image. If not, please select your topic:  /[$hostname]/display_xy/image/raw/draw_xy_line

Note: [$hostname] is your hostname on Jetbot_ros.

Next, open another terminal and type the code below"

```sh
$ rosrun rqt_reconfigure rqt_reconfigure
```

Then you can see the window "rqt_reconfigure__Param". Select "display_xy" from left function table, you can see five parametesr. There are introducations below:

* X:  to compute an approximate steering value wiht "Y"
* Y:  to compute an approximate steering value with "X"
* picture_interval: how many picture do you take in once
* save_image: enable this to save image number of picture_interval, and will auto disable after takes.
* image_number: how many pictures in [../catkin_ws/src/deep_learning/road_following/image/dataset_xy] now.

Now you can see the image in window "rqt_image_view", and use window "rqt_reconfigure__Param" to adjust parameter and save images at the same time.
If you finish saved, please press [ctrl] + [c] in any terminal for closing correctly.

## Step2. Training

Befor training the model, we'll configure the parameter about the model. Please check out and modify the file  [ ../catkin_ws/src/deep_learning/road_following/param/train_model.yaml ].

Note: Now we just support the model struct is "resnet18". And please check out the save_model_name, will not be the same as any model in [ ../catkin_ws/src/deep_learning/road_following/model ]. If your model name is repeat, system will auto add time after your model name.  

When you finished modified parameter about training model, you can type the code below to train model:

```sh
$ roslaunch road_following train_model.launch
```

In this example, information is:

> [INFO] [1614051323.337509]: [/Jetbot_ros01/train_model]  Initializing train_model.py......

> ...

> [INFO] [1614051359.547631]: [/Jetbot_ros01/train_model] Epoch: 1, train_loss: 1.72967553139, ... 

> ...

> [WARN] [1614051376.522800]: [/Jetbot_ros01/train_model] Now you can press [ctrl] + [c] ro close the launch file.
[Jetbot_ros01/train_model-2] process has died [pid 28548, exit code -11, cmd /home/icshop/Jetbot_ros/catkin_ws/src/deep_learning/road_following/src/train_model.py __name:=train_model __log:=/home/icshop/.ros/log/25746c46-7588-11eb-9e8a-a4c3f0eb8755/Jetbot_ros01-train_model-2.log].
log file: /home/icshop/.ros/log/25746c46-7588-11eb-9e8a-a4c3f0eb8755/Jetbot_ros01-train_model-2*.log

Great! Because the node will shutdown automatically, the last information will use "red text" to tell you there is an error occur. Dont't worry, if you can see the text "Now you can press [ctrl] + [c] to shutdwon the lauch file.", means training done and successfully. Just press [ctrl] + [c] to cancel the code in terminal "train_model.launch".

Now you can find the trained model in [ ../catkin_ws/src/deep_learning/road_following/model ], and know information for this trained model in [ ../catkin_ws/src/deep_learning/road_following/model/recording.yaml ].

## Step3. inference model and development

Before inferencing the model, we need to modify the param in [ ../catkin_ws/src/deep_learning/road_following/param/inference_model.yaml ]. 
Please check out the "model_pth", must be the same as one of the model in [ ../catkin_ws/src/deep_learning/road_following/model ].    

And then, you can type the code below to start inferencing the model and let Jetbot_ros to move:

```sh
$ roslaunch road_following inference.launch reaction"=true
```

In this example, when you see the information below, means Jetbot_ros can find the lane!

> ...

> [INFO] [1614052668.313290]: [/Jetbot_ros01/road_model_inference] Deployment complete! Use 14.64 seconds.

> [INFO] [1614052668.387532]: [/Jetbot_ros01/road_model_inference]  You can listen the topic to see how much the angle of head between the lane: /Jetbot_ros01/road_model_inference/inference

> ...

You can listen the topic to see what message in another terminal:

```sh
$ rostopic echo /[$hostname]/road_model_inference/inference
```

Note: [$hostname] is your hostname on Jetbot_ros

> angle: 1.4863204277

> angle_last: 1.4863204277

Awesome! Now you can start "another computer", and open new Terminal for typing the code below to adjust the parameter for PID controller:

```sh
$ rosrun rqt_reconfigure rqt_reconfigure
```
You can select "inverse_kinematics_twist_node" from left function table, then you can see the seven parameter, these about robot's motion. There are the introducations below:

* speed_gain: to start Jetbot_ros increase speed_gain_slider
* steer_gain: If you see Jetbot_ros is wobbling, you need to reduce steering_gain_slider till it is smooth
* trim: adjust speed between right and left motor
* k: speed correction value for trim
* limit: speed limit
* radius: wheels's radius
* save_parameter: If this parameter is perfect, you can select it to save in [../catkin_ws/src/jetbot_ros/param/$hostname_pid.yaml]

You can select "road_inference_to_reaction" from left function table, then you can see the five parameter, theese about PID's motion There are the introducations below:

* speed_gain: to start Jetbot_ros increase speed_gain_slider
* steering_gain and steering_kd: If you see Jetbot_ros is wobbling, you need to reduce steering_gain_slider till it is smooth
* steering_bias: If you see Jetbot_ros is biased towards extreme right or extreme left side of the track, you should control this slider till JetBot start following line or track in the center. This accounts for motor biases as well as camera offsets
* save_parameter: If this parameter is perfect, you can select it to save in [../catkin_ws/src/deep_learning/road_following/param/$hostname_pid.yaml]

If you want to stop the code, remember use pressing [ctrl] + [c] in any terminal for closing correctly.

# Use package self_driving
----
If you have finished package img_recognition and road_following, you can use package self_driving now.
Place your Jetbot_ros on your map with line and some obstacles, then you can type the code below:

```sh
$ roslaunch self_driving inference.launch reaction:=true
```

After deploy model, Jetbot_ros will start! You also can use the code below to adjust the parameter:

```sh
$ rosrun rqt_reconfigure rqt_reconfigure
```
You can select "inverse_kinematics_twist_node" from left function table, then you can see the seven parameter, these about robot's motion, and select "self_driving_inference_to_reaction" from left function table, then you can see the five parameter as the same as "road_inference_to_reaction".
If you select the parameter "save_parameter", it will save in [../catkin_ws/src/deep_learning/road_following/param/$hostname_pid.yaml.]

Note: "$hostname" is the hostname on your Jetbot_ros.

If you want to define your Jetbot_ros's motion, please use [../catkin_ws/src/deep_learning/self_deiving/src/sef_driving_inerence_to_reaction] to define.

If you want to stop it, remember use pressing [ctrl] + [c] in any terminal for closing correctly.