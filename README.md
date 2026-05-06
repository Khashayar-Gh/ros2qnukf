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

## Demo


<!-- **Video**: -->

<video src="media/ros2qnukf_demo.mp4" controls width="720"></video>

<!-- **GIF** (inline loop):

![Demo loop](media/ros2qnukf_demo.gif) -->

## Current Status

This implementation is functional for ROS 2 dataset bringup and localization experiments.

Current visual update uses **pseudo visual measurements** derived from GT-aligned synthetic points.  
This is intentional for the current phase and will be replaced by **stereo triangulation-based measurements** in later iterations.

## Assumptions and Limitations

- Target distro is **ROS 2 Jazzy**.
- Current measurement model uses pseudo visual landmarks, not true online feature triangulation yet.
- EuRoC-style bag + GT CSV workflow is the primary tested path.
- Launch defaults resolve inside the package via `$(find-pkg-share ros2qnukf)`. Drop your bag and GT CSV under `dataset/<name>/` and `dataset/<name>.csv`, or override `bag_path` / `path_gt_csv` on the command line.

## Requirements

### System

- Ubuntu 24.04.
- **ROS 2 Jazzy**.
- C++17 toolchain (GCC 11+ or Clang 14+).
- CMake ≥ 3.8, `colcon`, `ament_cmake`.

### Build dependencies

Resolved via `rosdep` from [package.xml](package.xml):

- `eigen` (Eigen3 ≥ 3.3)
- `rclcpp`
- `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `visualization_msgs`
- `message_filters`
- `tf2_ros`

### Runtime / launch dependencies

- `launch`, `launch_ros`, `launch_xml`
- `ros2 bag` (`ros2bag` + a storage plugin — `rosbag2_storage_mcap` or `rosbag2_storage_default_plugins`)
- `rviz2` (only if `rviz_enable:=true`)

### Data

- From [OpenVINS EuRoC dataset page](https://docs.openvins.com/gs-datasets.html#gs-data-euroc), download both:
  - ROS 2 bag data (`rosbag2`)
  - Ground-truth CSV data
- `ros2qnukf` already includes committed GT for `V2_01_easy` (`dataset/V2_01_easy.csv`), so you only need to download rosbag2 if you use default dataset naming.
- An EuRoC-style ROS 2 bag (default expects `dataset/V2_01_easy/`).
- A matching EuRoC GT CSV (default expects `dataset/V2_01_easy.csv`).

## Reproduce

### 1) Install dependencies

`colcon build` only compiles — it does **not** install system packages, so the deps above must be present before building. If you installed `ros-jazzy-desktop`, almost all of them are already on your system; the easiest way to fill in anything missing (notably `libeigen3-dev`) is `rosdep`, run from the workspace root (the directory that contains `src/`):

```bash
sudo rosdep init   # first-time setup only
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

This reads [package.xml](package.xml) and `apt install`s whatever is missing.

### 2) Build

From workspace root (directory containing `src/`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ros2qnukf
source install/setup.bash
```

### 3) Run (full launch: bag + nodes + optional RViz)

If your bag and CSV live under `dataset/<name>/` and `dataset/<name>.csv` inside the package, no overrides are needed — just pick the dataset name (defaults to `V2_01_easy`):

```bash
ros2 launch ros2qnukf ros2qnukf_ingestion.launch.xml dataset:=V2_01_easy
```

Otherwise, point the launch at any external bag + CSV:

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

