# Color-Based Visual Servoing for TurtleBot3 (`color_nav`)

A complete ROS 2 (Humble) package that enables a TurtleBot3 Burger robot to autonomously detect, track, and approach a green sphere in a Gazebo Classic simulation environment. The system fuses camera perception (HSV segmentation) with 2-D LiDAR proximity sensing to achieve safe, closed-loop navigation.
<img width="1179" height="754" alt="image" src="https://github.com/user-attachments/assets/b4667187-111e-4e8a-b249-e5ab9b1cbd0c" />

---

# 1. System Requirements & Dependencies

This package is designed and tested for the following environment:

* **OS:** Ubuntu 22.04 (Jammy Jellyfish)
* **Middleware:** ROS 2 Humble Hawksbill (Desktop Install)
* **Simulator:** Gazebo Classic 11
* **Robot Model:** TurtleBot3 Burger

## Core Package Dependencies

* `rclpy`: ROS 2 Python client library
* `sensor_msgs`: Image and LaserScan message types
* `geometry_msgs`: Twist command message types
* `cv_bridge`: ROS ↔ OpenCV image conversion bridge
* `gazebo_ros`: Simulation bridge
* `turtlebot3_gazebo`: Robot model and plugins
* **Python Libraries:**

  * `opencv-python >= 4.5`
  * `numpy >= 1.21`

---

# 2. Technical Architecture

The system is built on a modular architecture separating simulation physical properties, perception pipelines, and control logic.

## 2.1 Gazebo Simulation Setup (`green_sphere.world`)

The target is a green sphere defined using the Simulation Description Format (SDF) to ensure realistic physical interactions within the ODE physics engine.

### Simulation Properties

* **Mass:** `1.0 kg`
* **Inertia Tensor:**

  * `I_xx = 0.4`
  * `I_yy = 0.4`
  * `I_zz = 0.4`
* **Collision Geometry:** Sphere radius = `0.3 m`
* **Material:** `Gazebo/Green`
* **Spawn Position:**

  * `x = 2.0`
  * `y = 0.0`
  * `z = 0.5`

The object spawns approximately 2 meters in front of the robot.

---

## 2.2 ROS 2 Node Interface (`visual_servoing.py`)

The main node (`VisualServoing`) handles asynchronous sensor callbacks.

### Subscribers

| Topic               | Message Type            | Purpose          |
| ------------------- | ----------------------- | ---------------- |
| `/camera/image_raw` | `sensor_msgs/Image`     | RGB image stream |
| `/scan`             | `sensor_msgs/LaserScan` | 360° LiDAR scan  |

### Publishers

| Topic      | Message Type          | Purpose                              |
| ---------- | --------------------- | ------------------------------------ |
| `/cmd_vel` | `geometry_msgs/Twist` | Differential-drive velocity commands |

### Image Conversion

`CvBridge` converts ROS images into OpenCV matrices using:

```python
bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
```

This ensures compatibility with OpenCV’s native BGR channel ordering.

---

## 2.3 Computer Vision Pipeline (OpenCV)

The tracking pipeline uses HSV segmentation instead of raw BGR thresholding to improve robustness against lighting variations.

### Processing Pipeline

1. Convert image from BGR → HSV
2. Apply HSV threshold mask
3. Apply erosion and dilation
4. Extract contours
5. Compute centroid from largest contour
6. Calculate horizontal alignment error

### HSV Thresholds

```python
LOWER_GREEN = [40, 50, 50]
UPPER_GREEN = [80, 255, 255]
```

### Morphological Operations

```python
kernel = np.ones((5,5), np.uint8)
mask = cv2.erode(mask, kernel, iterations=1)
mask = cv2.dilate(mask, kernel, iterations=2)
```

### Contour Extraction

```python
cv2.findContours(mask,
                 cv2.RETR_EXTERNAL,
                 cv2.CHAIN_APPROX_SIMPLE)
```

### Centroid Mathematics

For the largest contour:

```python
cx = M["m10"] / M["m00"]
cy = M["m01"] / M["m00"]
```

Horizontal error:

```python
error_x = image_center_x - cx
```

Contours below `500 px²` are ignored as noise.

---

## 2.4 Control Strategy & Finite State Machine (FSM)

The robot operates using a 4-state finite state machine and a proportional controller.

## FSM States

### SEARCH

Activated when no contours are detected.

Robot behavior:

```python
angular.z = 0.25
linear.x = 0.0
```

The robot rotates counterclockwise until the target enters the camera field of view.

---

### ALIGN

Activated when the object exists but horizontal error exceeds the alignment threshold.

Controller equation:

```python
omega_z = kp * error_x
```

Angular velocity is clamped:

```python
-1.0 <= omega_z <= 1.0
```

Linear velocity remains zero during alignment.

---

### APPROACH

Activated once:

```python
abs(error_x) < alignment_threshold
```

Robot behavior:

```python
linear.x = forward_speed
angular.z = kp * error_x
```

The robot continuously re-centers while moving forward.

---

### STOP (Highest Priority)

Triggered by LiDAR proximity constraints.

All motion commands are overridden:

```python
linear.x = 0.0
angular.z = 0.0
```

---

## 2.5 LiDAR Proximity Fusion (Safety Gate)

<img width="1237" height="711" alt="image" src="https://github.com/user-attachments/assets/0ef9fc82-ded1-46e7-9f01-bdcc631c28d5" />


The monocular camera cannot estimate true depth, so LiDAR is fused as a safety layer.

### LiDAR Sector Extraction

Forward-facing ±15° sector:

```python
front_ranges = scan.ranges[-15:] + scan.ranges[:15]
```

### Validation

Invalid values are filtered:

```python
math.isinf(value)
math.isnan(value)
```

### Safety Condition

```python
if min_distance < safe_distance:
    stop_robot()
```

Default safety threshold:

```python
safe_distance = 2.5
```

---

# 3. Installation & Build Instructions

## Clone Repository

```bash
mkdir -p ~/peppermint_ws/src
cd ~/peppermint_ws/src
git clone (https://github.com/Enavamsi/Peppermint-Robotics-Challenge.git)
```

---

## Build Workspace

```bash
cd ~/peppermint_ws
colcon build --packages-select color_nav
```

---

## Source Workspace

```bash
source install/setup.bash
```

---

# 4. Usage & Configuration

## Standard Launch

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch color_nav simulation_launch.py
```

The launch file simultaneously starts:

* Gazebo Classic
* Robot State Publisher
* Spawn Entity Node
* Visual Servoing Node

---

## Launch Parameters

| Parameter             | Type  | Default  | Description                   |
| --------------------- | ----- | -------- | ----------------------------- |
| `kp`                  | float | `0.0025` | Proportional gain             |
| `alignment_threshold` | int   | `40`     | Pixel tolerance for alignment |
| `search_speed`        | float | `0.25`   | SEARCH angular speed          |
| `forward_speed`       | float | `0.05`   | APPROACH linear speed         |
| `safe_distance`       | float | `2.5`    | LiDAR stop distance           |
| `show_debug_window`   | bool  | `true`   | Enables OpenCV visualization  |

---

## Example Parameter Override

```bash
ros2 launch color_nav simulation_launch.py \
kp:=0.003 \
safe_distance:=2.0 \
forward_speed:=0.08
```

---

## Runtime Parameter Tuning

```bash
ros2 param set /visual_servoing kp 0.004
ros2 param set /visual_servoing safe_distance 1.8
```

---

# 5. Testing

The package includes a `pytest` suite for validating perception and control logic.

## Run Tests

```bash
colcon test --packages-select color_nav
colcon test-result --verbose
```

---

## Test Coverage

### 1. LiDAR Callback Validation

Verifies filtering of:

* `inf`
* `NaN`
* invalid ranges

---

### 2. Vision Callback Validation

Injects synthetic green images to verify:

* contour extraction
* centroid detection
* FSM transitions

---

### 3. Controller Constraint Validation

Verifies angular velocity clamping:

```python
-1.0 <= omega_z <= 1.0
```

under extreme alignment errors.

---

# 6. Known Limitations & Future Work

## Current Limitations

### Hardcoded HSV Bounds

HSV thresholds must currently be modified in source code.

**Proposed Improvement:**

* Dynamic parameter tuning
* GUI color picker
* runtime calibration

---

### Single-Axis Tracking

Only yaw error is considered.

**Proposed Improvement:**

* Add pitch-axis control
* Support pan-tilt camera systems

---

### P-Only Controller

Small residual tracking errors may remain.

**Proposed Improvement:**

* Upgrade to PI/PID controller
* Add anti-windup constraints

---

### Fixed LiDAR Sector

Forward cone is hardcoded to ±15°.

**Proposed Improvement:**

Expose:

```python
lidar_sector_deg
```

as a ROS parameter.

---

### No Object Identity Discrimination

Any sufficiently large green object becomes a target.

**Proposed Improvement:**

* ArUco marker recognition
* object classification
* blob identity tracking

---

# 7. Summary

`color_nav` demonstrates a complete autonomous visual-servoing pipeline for TurtleBot3 using:

* ROS 2 Humble
* OpenCV-based HSV segmentation
* LiDAR safety fusion
* finite state machine navigation
* proportional closed-loop control

The package provides a strong foundation for extending into:

* autonomous manipulation
* target following
* mobile robotics research
* multi-sensor fusion systems
* adaptive visual navigation
