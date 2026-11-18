# mocap4ros2_qualisys

This repository provides ROS 2 integration for the Qualisys Motion Capture System through two packages:

- **`qualisys_driver`**: The original ROS 2 driver for streaming motion capture data from Qualisys QTM.
- **`qualisys_ros2_control_hw`**: A ROS 2 Control hardware interface exposing selected Qualisys rigid bodies as 6-DOF pose sensors.

## qualisys_driver

[![GitHub Action Status](https://github.com/MOCAP4ROS2-Project/mocap4ros2_qualisys/actions/workflows/main.yaml/badge.svg)](https://github.com/MOCAP4ROS2-Project/mocap4ros2_qualisys/actions/workflows/main.yaml)

[![codecov](https://codecov.io/gh/MOCAP4ROS2-Project/mocap_optitrack_driver/main/graph/badge.svg)](https://codecov.io/gh/MOCAP4ROS2-Project/mocap_optitrack_driver)

Requisites: qualisys_cpp_sdk

> [!IMPORTANT]
> Currently, only `rt_protocol_1.23` is supported.

Create workspace:
```
mkdir -p mocap_ws/src && cd mocap_ws/src
```

Download qualisys repo recursively to get the Qualisys C++ SDK, or set `QualisysSDK_PATH`.

> [!NOTE]
> CMake downloads and installs qualisys sdk automatically using fetch content.

```
git clone --recursive https://github.com/MOCAP4ROS2-Project/mocap4ros2_qualisys.git
```

Install dependencies:
```
vcs import < mocap4ros2_qualisys/dependency_repos.repos
```

Compiling workspace:
```
cd .. && colcon build --symlink-install
```

Source workspace:
```
source install/setup.bash
```

Setup your qualisys configuration:
```
mocap_ws/src/mocap4ros2_qualisys/qualisys_driver/config/qualisys_driver_params.yaml
```

Launch qualisys system:
```
ros2 launch qualisys_driver qualisys.launch.py
```

Visualize in RViz:
```
ros2 launch mocap_marker_viz mocap_marker_viz.launch.py mocap_system:=qualisys
```

## qualisys_ros2_control_hw

This package provides a ROS 2 Control hardware interface for the Qualisys Motion Capture System.

It allows users to select rigid bodies by their labels in the Qualisys system and expose their 6-DOF poses as ROS 2 Control state interfaces, which can be accessed through standard ROS 2 Control controllers, such as the `pose_broadcaster`.

For installation, configuration, and usage instructions, see the [qualisys_ros2_control_hw README](qualisys_ros2_control_hw/README.md).