# color_nav

> Colour-based visual servoing for TurtleBot3 — ROS 2 Humble / Gazebo Classic

---

## Overview

`color_nav` is a **single self-contained ROS 2 package** that demonstrates
colour-guided navigation using a TurtleBot3 Burger robot.

| Feature | Detail |
|---|---|
| Target detection | HSV segmentation (green sphere) |
| Controller | Proportional (P) angular controller |
| Safety | LiDAR proximity stop |
| Behaviour | Finite-state machine (SEARCH → ALIGN → APPROACH → STOP) |
| Simulation | Gazebo Classic, custom world bundled |

---

## Package Structure

```
color_nav/
├── color_nav/
│   ├── __init__.py
│   └── visual_servoing.py      ← main node
├── launch/
│   └── simulation.launch.py    ← full-stack launch
├── worlds/
│   └── green_sphere.world      ← Gazebo world
├── test/
│   └── test_visual_servoing.py ← unit tests
├── resource/color_nav
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Dependencies

| Package | Purpose |
|---|---|
| `rclpy` | ROS 2 Python client |
| `sensor_msgs` | Image, LaserScan |
| `geometry_msgs` | Twist |
| `cv_bridge` | ROS ↔ OpenCV conversion |
| `python3-opencv` | HSV segmentation |
| `python3-numpy` | Array operations |
| `turtlebot3_gazebo` | Robot model & simulation |
| `gazebo_ros` | Gazebo–ROS bridge |

---

## Build

```bash
cd ~/peppermint_ws
colcon build --packages-select color_nav
source install/setup.bash
```

---

## Run

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch color_nav simulation.launch.py
```

### Override parameters at launch

```bash
ros2 launch color_nav simulation.launch.py \
    kp:=0.003 \
    safe_distance:=2.0 \
    forward_speed:=0.08 \
    show_debug_window:=false
```

### Tune at runtime (no restart needed)

```bash
ros2 param set /visual_servoing kp 0.004
ros2 param set /visual_servoing safe_distance 1.8
```

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `kp` | float | `0.0025` | Proportional gain |
| `alignment_threshold` | int | `40` | Pixel error to enter APPROACH [px] |
| `search_speed` | float | `0.25` | Rotation speed when target lost [rad/s] |
| `forward_speed` | float | `0.05` | Linear speed when aligned [m/s] |
| `safe_distance` | float | `2.5` | LiDAR stop distance [m] |
| `hsv_lower` | int[3] | `[40,50,50]` | Lower HSV bound |
| `hsv_upper` | int[3] | `[80,255,255]` | Upper HSV bound |
| `show_debug_window` | bool | `true` | Show OpenCV windows |
| `min_contour_area` | float | `500.0` | Noise filter [px²] |
| `lidar_sector_deg` | int | `15` | Front LiDAR half-angle [°] |

---

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Subscribed |
| `/scan` | `sensor_msgs/LaserScan` | Subscribed |
| `/cmd_vel` | `geometry_msgs/Twist` | Published |

---

## Finite State Machine

```
         ┌─────────────────────────────────────────┐
         │             obstacle detected           │
         ▼                (any state)              │
      ┌──────┐                                     │
      │ STOP │ ◄───────────────────────────────────┘
      └──────┘

      ┌────────┐   blob found   ┌───────┐  aligned  ┌──────────┐
      │ SEARCH │ ─────────────► │ ALIGN │ ─────────► │ APPROACH │
      └────────┘                └───────┘            └──────────┘
          ▲                         │                      │
          └─────────────────────────┴──── blob lost ───────┘
```

---

## Tests

```bash
colcon test --packages-select color_nav
colcon test-result --verbose
```

---

## Licence

Apache-2.0 — see `package.xml`
