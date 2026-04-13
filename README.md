# Dynamic Obstacle Prediction

A ROS package for robust detection, tracking, and prediction of dynamic obstacles using sensor fusion (LiDAR + RGB-D camera) with adaptive noise filtering for mobile robot navigation.

## Author
**Pranav Chintalapudi**
January 2026

## Overview

This package implements a multi-stage filtering pipeline that distinguishes between truly moving obstacles (pedestrians, vehicles) and false positives caused by robot motion, sensor noise, and environmental effects.

### Key Features

- **Sensor Fusion**: Combines 2D LiDAR and 3D camera point clouds for robust detection
- **Context-Aware Filtering**: Adapts noise rejection based on robot motion state (stationary/moving/rotating)
- **Parallel Motion Detection**: Identifies objects moving with the robot vs. stationary objects affected by ego-motion
- **Velocity Prediction**: Estimates obstacle velocities with exponential smoothing and outlier rejection
- **Dual-Mode Classification**: Different thresholds for stationary vs. moving robot states
- **Real-time Performance**: Optimized for online execution at 10-20 Hz

## System Architecture

```
┌─────────────┐      ┌──────────────┐
│ LiDAR Scan  │      │ Camera Depth │
└──────┬──────┘      └──────┬───────┘
       │                    │
       └────────┬───────────┘
                │
         ┌──────▼───────┐
         │ Clustering & │
         │   Detection  │
         └──────┬───────┘
                │
         ┌──────▼───────┐
         │     Data     │
         │ Association  │
         └──────┬───────┘
                │
    ┌───────────▼────────────┐
    │  Multi-Stage Filtering │
    ├────────────────────────┤
    │ 1. Velocity Validation │
    │ 2. Acceleration Limit  │
    │ 3. Noise Gate          │
    │ 4. Parallel Detection  │
    │ 5. Smoothing           │
    └───────────┬────────────┘
                │
         ┌──────▼───────┐
         │Classification│
         │ State Machine│
         └──────┬───────┘
                │
         ┌──────▼───────┐
         │ Prediction & │
         │Visualization │
         └──────────────┘
```

## Installation

### Prerequisites
- ROS Noetic (Ubuntu 20.04)
- PCL 1.10+
- Eigen3

### Build Instructions

```bash
cd ~/catkin_ws/src
git clone <repository_url> dynamic_obstacle_prediction
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### Launch the Node

```bash
roslaunch dynamic_obstacle_prediction obstacle_predictor.launch
```

### Configuration

Edit `config/obstacle_predictor.yaml` to tune parameters:

**Critical Parameters:**
- `unknown_to_dynamic_speed_stationary`: 0.30 m/s (detection sensitivity when robot is still)
- `unknown_to_dynamic_speed_moving`: 0.60 m/s (detection sensitivity when robot is moving)
- `parallel_motion_alignment_threshold`: 0.85 (strictness of parallel motion detection)
- `base_noise_gate`: 0.08 m (sensor noise floor)

See `docs/PARAMETERS.md` for detailed parameter descriptions.

### Topics

**Subscribed:**
- `/scan` (sensor_msgs/LaserScan) - 2D LiDAR data
- `/camera/depth/points` (sensor_msgs/PointCloud2) - RGB-D camera depth
- `/odom` (nav_msgs/Odometry) - Robot odometry

**Published:**
- `/dynamic_obstacles_markers` (visualization_msgs/MarkerArray) - Detected obstacles
- `/predicted_obstacle_paths` (visualization_msgs/MarkerArray) - Predicted trajectories

### Visualization

View in RViz:
```bash
rviz -d config/obstacle_predictor.rviz
```

## Algorithm Details

### Multi-Stage Filtering Pipeline

**Stage 1: Velocity Plausibility**
- Rejects velocities > 12 m/s (pedestrian maximum)

**Stage 2: Acceleration Limiting**
- Context-aware thresholds:
  - Stationary robot: 3.0 m/s²
  - Moving robot: 8.0 m/s²
  - Rotating robot: 20.0 m/s² (high tolerance for perspective effects)

**Stage 3: Adaptive Noise Gate**
```
noise_threshold = base + distance × ω × scale_factor
```
- Compensates for tangential velocity during rotation
- Distance-dependent noise modeling

**Stage 4: Parallel Motion Detection**
```
alignment = dot(robot_direction, object_direction)
speed_ratio = object_speed / robot_speed

if alignment > 0.85 AND 0.5 < speed_ratio < 2.0:
    → True moving object
```

**Stage 5: Exponential Smoothing**
```
v_filtered = α × v_instant + (1-α) × v_previous
```
- Adaptive α based on track age and robot motion state

### Classification State Machine

```
UNKNOWN ──────────> DYNAMIC  (sustained motion detected)
   │                   │
   │                   │
   └────> STATIC <─────┘      (motion ceases)
```

**Dual-Mode Thresholds:**
- Robot stationary → Lower thresholds (catch slow movers)
- Robot moving → Higher thresholds (reject noise)

## Performance

**Tested Configuration:**
- AMR Platform: AR250 (0.12 m/s linear, 0.25 rad/s angular)
- Sensors: 2D LiDAR (270°, 10Hz) + Intel RealSense D435i
- Detection Range: 0.4m - 8.0m
- Update Rate: 10 Hz
- CPU Usage: ~15% (Intel i5-8250U)

**Detection Performance:**
- True Positive Rate: 95% (moving pedestrians)
- False Positive Rate: <2% (stationary objects during motion)
- Velocity Estimation Error: ±0.15 m/s (stationary robot), ±0.25 m/s (moving robot)

## Troubleshooting

### Issue: False positives during robot rotation
**Solution:** Increase `parallel_motion_alignment_threshold` to 0.90 or `max_acceleration_rotating` to 25.0

### Issue: Slow-moving objects not detected
**Solution:** Decrease `unknown_to_dynamic_speed_stationary` to 0.25 and `base_noise_gate` to 0.06

### Issue: Objects keep switching between STATIC/DYNAMIC
**Solution:** Increase `min_age_for_classification` to 12 and `velocity_recovery_factor` to 0.80

See `docs/TROUBLESHOOTING.md` for more solutions.

## Project Structure

```
dynamic_obstacle_prediction/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── obstacle_predictor.yaml
│   └── obstacle_predictor.rviz
├── docs/
│   ├── ALGORITHM.md
│   ├── PARAMETERS.md
│   └── TROUBLESHOOTING.md
├── include/dynamic_obstacle_prediction/
│   └── ObstaclePredictor.h
├── launch/
│   └── obstacle_predictor.launch
└── src/
    ├── ObstaclePredictor.cpp
    └── ObstaclePredictorNode.cpp
```

