# Jetson Nano

<p align="center">
  <img src="https://github.com/kjoelovelife/Jetson_nano/blob/master/icshop_jetson_nano.jpg" width="280">
</p>

A collection of projects and tools for the NVIDIA Jetson Nano, covering robot control, computer vision, deep learning, and ROS integration.

> If you purchased the kit from iCShop, the default image password is `075564686`.

## Contents

| Directory | Description |
|-----------|-------------|
| `install_script/` | Setup scripts for the Jetson Nano environment |
| `jetbot/` | JetBot robot Python package (motors, camera, Jupyter notebooks) |
| `jetcam/` | Camera library for CSI and USB cameras via GStreamer |
| `jetbot_ros/` | ROS Melodic workspace with motor control, camera, and deep learning nodes |
| `OpenCV-Face-Recognition/` | Real-time face detection and recognition with OpenCV |
| `Machine_Learning/` | Classification and regression dataset utilities |
| `jetson-dlinano/` | NVIDIA Deep Learning Institute course notebooks |
| `driver/` | WiFi adapter drivers (rtl8812au, EW-7822ULC) |

## Quick Start

Clone this repository to your home directory as `Jetson_nano`:

```bash
git clone https://github.com/kjoelovelife/Jetson_nano ~/Jetson_nano
cd ~/Jetson_nano
```

Run the interactive installer to set up the components you need:

```bash
./install.sh
```

The installer prompts for each component individually:
- Swap space (4 GB, recommended before any build)
- SSH configuration
- OpenCV 4.3.0 with CUDA support
- jetson-inference
- JetBot Python package + Jupyter service
- JetCam Python package
- ROS Melodic

## Requirements

- NVIDIA Jetson Nano Developer Kit
- JetPack 4.4 SD card image
- 32 GB or larger microSD card (64 GB recommended for full installation)

## Projects

### JetBot (Standalone)

Interactive robot control through Jupyter notebooks. Supports teleoperation, collision avoidance, road following, and object following.

```bash
cd install_script && sudo ./jetbot_install.sh
```

### JetBot ROS

ROS Melodic integration for the JetBot. Requires ROS Melodic, OpenCV 4.3.0, and PyTorch 1.4.0.

```bash
cd install_script && ./ros_install_melodic.sh
cd ~/Jetson_nano/jetbot_ros && ./install/catkin_make.sh
source setup/environment.sh
```

See [`jetbot_ros/README.md`](jetbot_ros/README.md) for node usage.

### OpenCV Face Recognition

Real-time face recognition pipeline using Haar cascades and LBPH. Works with USB and CSI (Raspberry Pi) cameras.

```bash
cd OpenCV-Face-Recognition && ./face_recognition_dependencies.sh
# Then run scripts 01 → 02 → 03 in the USB_camera or raspi_camera subdirectory
```

## Developer

- [Joe Lin](mailto:weichih.lin@protonmail.com)

## References

- [Jetson Nano Getting Started](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit)
- [JetPack Installation](https://docs.nvidia.com/jetson/jetpack/install-jetpack/index.html)
- [NVIDIA JetBot](https://github.com/NVIDIA-AI-IOT/jetbot)
- [jetson-inference](https://github.com/dusty-nv/jetson-inference)
- [TensorRT](https://developer.nvidia.com/tensorrt)
- [Jetson Nano wiki (elinux)](https://elinux.org/Jetson_Nano)
