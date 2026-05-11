# Peppermint-Robotics-Challenge

# Color-Based Visual Servoing for TurtleBot3 (`color_nav`)

A complete ROS 2 (Humble) package that enables a TurtleBot3 Burger robot to autonomously detect, track, and approach a green sphere in a Gazebo Classic simulation environment. The system fuses camera perception (HSV segmentation) with 2-D LiDAR proximity sensing to achieve safe, closed-loop navigation.

## Features

* **Robust Color Tracking:** Uses HSV color space thresholding and morphological operations to robustly segment a green target, immune to simulated lighting variations.
* **Proportional Control:** Implements a P-controller for smooth rotational alignment based on image moments and centroid tracking.
* **LiDAR Sensor Fusion:** Processes raw 360-degree `LaserScan` data to create an absolute safety stop gate, preventing collisions regardless of the vision state.
* **FSM Recovery State:** Automatically falls back into a `SEARCH` rotation pattern if the target moves out of the camera's field of view.
* **Highly Configurable:** Exposes control gains, speeds, and safety thresholds as launch parameters for easy runtime tuning.

## Prerequisites

This package is tested and designed for:
* **OS:** Ubuntu 22.04 (Jammy)
* **ROS Version:** ROS 2 Humble Hawksbill
* **Simulator:** Gazebo Classic 11

**System Dependencies:**
```bash
sudo apt update
sudo apt install ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs
pip3 install opencv-python numpy

Installation
Create a ROS 2 workspace (if you don't already have one) and clone the repository:

Bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone [https://github.com/YourUsername/Your-Repository-Name.git](https://github.com/YourUsername/Your-Repository-Name.git)
