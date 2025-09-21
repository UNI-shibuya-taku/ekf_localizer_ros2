# ekf_localizer_ros2

## Overview

## Environment
- ROS2 humble

## Install and Build
```
# clone repository
git clone https://github.com/UNI-shibuya-taku/ekf_localizer_ros2.git

# build
cd ~/colcon_ws
colcon build
```

## How to use
```
# Download the map data into the "pcd" folder and update the map data path in the launch file.

# run
ros2 launch ekf_localizer ekf_locali.launch.py
```
