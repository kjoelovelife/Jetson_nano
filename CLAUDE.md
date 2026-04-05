# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This repository is a collection of projects and tools for the **NVIDIA Jetson Nano** platform. It is organized into several independent sub-projects, not a single unified application.

## Sub-projects

| Directory | Purpose |
|-----------|---------|
| `jetbot/` | Standalone Python package for JetBot robot (Adafruit MotorHAT + camera, Jupyter-based) |
| `jetcam/` | Python camera library for CSI and USB cameras via GStreamer |
| `jetbot_ros/` | ROS Melodic workspace for JetBot with deep learning nodes |
| `OpenCV-Face-Recognition/` | Face detection and recognition scripts using OpenCV Haar cascades |
| `Machine_Learning/` | Classification and regression dataset/utility scripts |
| `jetson-dlinano/` | NVIDIA DLI course notebooks and Docker setup |
| `install_script/` | Individual component install scripts |
| `driver/` | WiFi adapter kernel drivers (rtl8812au, EW-7822ULC) |

## Installation

### Full automated install (Jetson Nano only)
```bash
# Run from ~/Jetson_nano — prompts for each component
./install.sh
```
This script sets the Nano to 10W mode (`nvpmodel -m0`) and optionally installs: swap (4G), SSH, OpenCV 4.3.0, jetson-inference, jetbot, jetcam, ROS Melodic.

### Individual components
```bash
cd install_script
./swap.sh                      # Configure 4G swap (recommended before building)
sudo ./opencv430_install.sh    # Build OpenCV 4.3.0 + contrib from source
./jetbot_install.sh            # Install JetBot Python package
./jetcam_install.sh            # Install JetCam Python package
./ros_install_melodic.sh       # Install ROS Melodic
./jetson_inference_install.sh  # Build jetson-inference
./ssh_setup.sh                 # Configure SSH
```

### Python packages (standalone)
```bash
# jetbot
cd jetbot && sudo python3 setup.py install

# jetcam
cd jetcam && sudo python3 setup.py install
```

## ROS Workspace (jetbot_ros)

**Environment:** JetPack 4.4, ROS Melodic, Python 2.7, PyTorch 1.4.0, OpenCV 4.3.0

### Build
```bash
# Build cv_bridge with Python 3 support first
cd ~/Jetson_nano/jetbot_ros
./install/catkin_make.sh
```
This builds `cvbridge_build_ws` (Python 3 cv_bridge), then `catkin_ws`, and appends `environment.sh` to `~/.bashrc`.

### Environment setup
```bash
source ~/Jetson_nano/jetbot_ros/setup/environment.sh
```
Sets `ROS_HOSTNAME`, `ROS_MASTER_URI` (to `$HOSTNAME.local:11311`), and sources the catkin workspace.

### Run nodes
```bash
roscore

# Motor driver (subscribes to ~wheels_cmd topic, WheelsCmd msg)
rosrun jetbot_ros wheels_driver_node.py

# OLED display
rosrun jetbot_ros jetbot_oled.py

# Camera stream (publishes to /jetbot_camera/raw as BGR8)
rosrun jetbot_ros jetbot_camera
```

### Multi-machine ROS setup
```bash
# On Jetson (vehicle)
source jetbot_ros/setup/set_ros_master.sh

# On laptop
cd jetbot_ros/catkin_ws/src/on_laptop/src
python teleop_key.py     # keyboard teleoperation
python teleop_joy.py     # joystick teleoperation
```

## Deep Learning (jetbot_ros)

The `jetbot_ros/catkin_ws/src/deep_learning/` package contains three pipelines, each following the same pattern: collect data → train model → run inference → react.

- `img_recognition/` — image classification (collect images → train ResNet → publish reaction)
- `road_following/` — regression-based road following (XY regression → steer)
- `self_driving/` — combined collision avoidance + road following

## OpenCV Face Recognition

Three-step pipeline (scripts numbered 01–04):
1. `01_face_dataset.py` — capture face images from camera
2. `02_face_training.py` — train LBPH recognizer, outputs `trainer/trainer.yml`
3. `03_face_recognition.py` — live recognition via webcam
4. `04_face_recognition_image.py` — recognition on a static image

Variants exist for USB camera (`USB_camera/`) and Raspberry Pi CSI camera (`raspi_camera/`).

```bash
# Install dependencies
cd OpenCV-Face-Recognition
./face_recognition_dependencies.sh
```

## Architecture Notes

- **jetbot/** uses `traitlets` for reactive programming — camera and motor values are traitlets observables; Jupyter widgets attach to them directly.
- **jetcam/** wraps GStreamer pipelines. `CSICamera` uses `nvarguscamerasrc`, `USBCamera` uses `v4l2src`. Both expose a `read()` method returning BGR8 numpy arrays, and support a `running=True` callback model.
- **jetbot_ros/** motor driver scales `[-1.0, 1.0]` speed to PWM `[0, 115]` via `Adafruit_MotorHAT` on I2C bus 1, channels 1 (left) and 2 (right). The `motor_alpha` parameter (default `-1.0`) flips motor direction.
- The `jetbot_ros/cvbridge_build_ws/` workspace is a separately built cv_bridge configured for Python 3, required because the system ROS Melodic ships with Python 2 cv_bridge.
