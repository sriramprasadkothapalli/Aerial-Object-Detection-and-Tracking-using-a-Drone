# Aerial-Object-Detection-and-Tracking-using-a-Drone

This project showcases an autonomous drone system capable of real-time object detection and intelligent path planning using deep reinforcement learning. The work was done as part of the Robot Learning course at the University of Maryland.

## 🚁 Overview
AutoNav is a two-phase project:

Phase 1: PX4-based drone system with YOLOv8 for object detection.

Phase 2: SJTU environment integrating PPO (Proximal Policy Optimization) for learning-based navigation along with YOLOv8.

## Goals
Detect objects in real-time using YOLOv8.

Perform autonomous path planning and obstacle avoidance using PPO.

Simulate and compare performance between PX4 and SJTU environments.

## 🧠 Technologies Used
YOLOv8 for object detection.

PPO (Reinforcement Learning via Stable-Baselines3).

ROS2 for communication.

PX4 SITL, Gazebo, MAVSDK (Phase 1).

SJTU DroneSim (Phase 2).

Python, OpenCV, PyTorch.

## ✅ Key Results
Feature	PX4 Phase	SJTU PPO Phase

Object Detection	YOLOv8 (on video stream)	YOLOv8 (integrated with PPO)

Path Planning	Manual or pre-defined	Learned using PPO

Obstacle Avoidance	Basic	Adaptive & Reward-driven

Integration Complexity	High	Streamlined

## Instructions

### Phase 1

After downloading PX4 Autopilot and required dependencies:

Terminal #1:
cd ~/PX4-Autopilot
export LIBGL_ALWAYS_SOFTWARE=1
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4

Terminal #2:
ros2 run ros_gz_bridge parameter_bridge \
/world/default/model/x500_depth_0/link/rgb_camera_link/sensor/rgb_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image \
/world/default/model/x500_depth_0/link/rgb_camera_link/sensor/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo

Terminal #3:
cd ~/px4_drone
python camera_yolov8_frames.py

Terminal #4:
cd ~/px4_drone
python drone_teleop.py

## Phase 2

Clone this repository into your ROS2 workspace

cd ~/ros2_ws/src
git clone <repository_url>

Navigate to the root of your workspace and build the package

cd ~/ros2_ws
colcon build 

### Source the workspace

source install/setup.bash

### Lauch the SJTU drone

ros2 launch sjtu_drone_description start_drone_launch.py

### Now the drone is spawned in the environment and now to take off the drone

ros2 topic pub /demo/takeoff std_msgs/msg/Empty {}

### To start the training of PPO

ros2 launch sjtu_drone_description start_training.launch.py

### To open the camera and detect objects using yolov8

ros2 launch sjtu_drone_description front_cam_detection.py


## 📌 Future Work
Real-world hardware testing.

Advanced object detection for complex terrains.

Faster PPO convergence.

Autonomous landing and multi-drone coordination.


## 📂 Repository Structure
├── px4_phase/          # PX4 SITL-based object detection

├── sjtu_phase/         # PPO and YOLO integration

├── models/             # YOLOv8 model files

├── scripts/            # PPO training and evaluation

├── PPO_test_RL.zip/            # Trained Model

├── README.md
