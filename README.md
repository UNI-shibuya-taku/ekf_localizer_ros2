# ekf_localizer_ros2

## Overview
EKF-based localization for autonomous mobile robots in ROS 2, fusing odometry, IMU, NDT scan matching and GNSS.

## Environment
- Ubuntu 22.04 / ROS 2 Humble
- CUDA (NDT matching runs on GPU via `fast_gicp::NDTCuda`)

## Dependencies
- [fast_gicp](https://github.com/SMRT-AIST/fast_gicp) (built with CUDA enabled)
- [ndt_omp](https://github.com/koide3/ndt_omp)
- PCL, Eigen, GeographicLib

```
sudo apt install ros-humble-pcl-ros ros-humble-tf2-eigen libgeographic-dev geographiclib-tools
```

## Install and Build
```
cd ~/colcon_ws/src
git clone https://github.com/UNI-shibuya-taku/ekf_localizer_ros2.git
cd ~/colcon_ws
colcon build --packages-select ekf_localizer
```

## How to use
1. Put your map (`.pcd`) somewhere and set `pcd_file_path` in `config/map_matcher/map_matcher.yaml`.
2. Set the initial pose (`INIT_*`) in `config/ekf/ekf.yaml`, and the GNSS origin (`gps_origin_*`) in `config/gps_updater/gps_updater.yaml`.
3. Launch:
```
ros2 launch ekf_localizer ekf_locali.launch.py
# with rosbag
ros2 launch ekf_localizer ekf_locali.launch.py use_sim_time:=true
```
The pose can be reset from RViz with "2D Pose Estimate" (`/initialpose`).

## Nodes
| Node | Description |
|---|---|
| `ekf_localizer_node` | EKF. Fuses odometry/IMU (prediction) with NDT/GNSS poses (correction) |
| `map_matcher_node` | NDT matching between LiDAR scan and the local map cropped around the EKF pose |
| `gps_updater_node` | Converts `NavSatFix` (lat/lon/alt) to a pose in the `map` frame |
| `tf_cub_node` | Broadcasts `map -> base_link` (or `map -> odom -> base_link`) TF from the EKF pose |

## Topics
### Subscribed
| Topic | Type | Node |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | ekf_localizer |
| `/imu/data_raw` | `sensor_msgs/Imu` | ekf_localizer |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | map_matcher |
| `/um982_node/fix_secondary` | `sensor_msgs/NavSatFix` | gps_updater, ekf_localizer |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | ekf_localizer, map_matcher, gps_updater |

### Published
| Topic | Type | Description |
|---|---|---|
| `/ekf_pose` | `geometry_msgs/PoseStamped` | Estimated pose (EKF output) |
| `/ndt_pose` | `geometry_msgs/PoseStamped` | NDT matching result |
| `/gps_pose` | `geometry_msgs/PoseWithCovarianceStamped` | GNSS pose in `map` frame |
| `map_out` | `sensor_msgs/PointCloud2` | Loaded PCD map |
| `/ndt_pc` | `sensor_msgs/PointCloud2` | Aligned LiDAR scan |
| `/local_map` | `sensor_msgs/PointCloud2` | Cropped local map (debug) |
| `/tf` | `tf2_msgs/TFMessage` | `map -> base_link` |

## Parameters
See the comments in each YAML file for details.

| File | Main parameters |
|---|---|
| `config/ekf/ekf.yaml` | Initial pose, sensor noise (`SIGMA_*`), outlier thresholds, enabling NDT/GNSS update, GNSS respawn |
| `config/map_matcher/map_matcher.yaml` | PCD path, voxel size, crop range, NDT resolution, score threshold |
| `config/gps_updater/gps_updater.yaml` | GNSS topic, origin (lat/lon/alt), covariance threshold |

## Directory structure
```
├── config/   # parameter files (per environment)
├── include/  # headers
├── launch/   # launch files
├── pcd/      # place map files here (optional)
└── src/      # node sources
```
