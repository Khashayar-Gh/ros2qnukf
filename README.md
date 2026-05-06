# ros2qnukf

`ros2qnukf` is a C++ / ROS 2 implementation of the Quaternion-based Unscented Kalman Filter (QNUKF) visual-inertial workflow.

This package is the ROS 2 C++ counterpart of:

- Original repository: [QNUKF](https://github.com/Khashayar-Gh/QNUKF)
- Paper: [A Quaternion-Based Unscented Kalman Filter for Accurate and Numerically Stable Orientation Estimation in Inertial and Visual Navigation Systems](https://doi.org/10.1109/TIM.2024.3509582)

## Highlights

- ROS 2 Jazzy native package (`rclcpp`, launch, params, TF, RViz).
- End-to-end EuRoC-style ingestion pipeline (IMU + stereo topics + GT CSV support).
- Numerically robust UKF update path in C++ with practical runtime safeguards.
- Reproducible launch flow for dataset playback, estimate publishing, and visualization.

## Current Status

This implementation is functional for ROS 2 dataset bringup and localization experiments.

Current visual update uses **pseudo visual measurements** derived from GT-aligned synthetic points.  
This is intentional for the current phase and will be replaced by **stereo triangulation-based measurements** in later iterations.

## Assumptions and Limitations

- Target distro is **ROS 2 Jazzy**.
- Current measurement model uses pseudo visual landmarks, not true online feature triangulation yet.
- EuRoC-style bag + GT CSV workflow is the primary tested path.
- Launch defaults include machine-specific paths; override them on your machine.

## Reproduce

### 1) Prerequisites

- Ubuntu with ROS 2 Jazzy installed.
- Colcon workspace with this package under `src/`.
- EuRoC-style ROS 2 bag and a matching GT CSV file.

### 2) Build

From workspace root (directory containing `src/`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ros2qnukf
source install/setup.bash
```

### 3) Run (full launch: bag + nodes + optional RViz)

```bash
ros2 launch ros2qnukf ros2qnukf_ingestion.launch.xml \
  bag_path:=/path/to/euroc_bag \
  path_gt_csv:=/path/to/euroc_gt.csv
```

Common useful overrides:

```bash
ros2 launch ros2qnukf ros2qnukf_ingestion.launch.xml \
  bag_path:=/path/to/euroc_bag \
  path_gt_csv:=/path/to/euroc_gt.csv \
  rviz_enable:=true \
  bag_rate:=1.0 \
  params_file:=/path/to/ros2qnukf_ingestion.params.yaml
```

### 4) Verify outputs

```bash
ros2 topic hz /ros2qnukf/pose_estimate
ros2 topic echo /ros2qnukf/pose_estimate --once
```

Expected core topics:

- `/ros2qnukf/pose_estimate`
- `/ros2qnukf/path_estimate`
- `/ros2qnukf/pose_gt`
- `/ros2qnukf/path_gt`

## Configuration Notes

- Main parameter file: `config/ros2qnukf_ingestion.params.yaml`
- Launch file: `launch/ros2qnukf_ingestion.launch.xml`

