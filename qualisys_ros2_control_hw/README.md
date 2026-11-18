# Qualisys ROS 2 Control Driver

This package provides a ROS 2 Control hardware interface for the Qualisys Motion Capture System, built on top of the `mocap4ros2_qualisys` repository.

It allows users to expose the 6-DOF poses of selected Qualisys rigid bodies as ROS 2 Control state interfaces. Rigid bodies are identified by their labels in the Qualisys system and configured through the robot's URDF/Xacro description.

## Features

- Connection to a Qualisys Motion Capture System through the Qualisys Real-Time Protocol (QTM RT Protocol).
- Support for multiple rigid bodies, selected by their Qualisys labels.
- Exposure of 3D position and quaternion orientation as ROS 2 Control state interfaces.
- Compatibility with ROS 2 Control pose broadcasters.
- Automatic conversion of position measurements from millimeters to meters.

## Configuration

### 1. Hardware Interface (URDF/Xacro)

The hardware interface is configured through a `<ros2_control>` block in the robot description.

Example:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="qualisys_sensor">
    <ros2_control name="qualisys_sensor" type="sensor">
      <hardware>
        <plugin>qualisys_ros2_control_hw/QualisysSensor</plugin>
        <param name="host_name">10.62.9.135</param>
        <param name="host_port">22222</param>
      </hardware>

      <sensor name="camera">
        <state_interface name="position.x"/>
        <state_interface name="position.y"/>
        <state_interface name="position.z"/>
        <state_interface name="orientation.x"/>
        <state_interface name="orientation.y"/>
        <state_interface name="orientation.z"/>
        <state_interface name="orientation.w"/>
        <param name="label">camera</param>
      </sensor>

    </ros2_control>
  </xacro:macro>

</robot>
```

**Hardware parameters:**

| Parameter | Description |
|---|---|
| `host_name` | IP address or hostname of the Qualisys QTM server |
| `host_port` | Port used for the Qualisys RT Protocol connection |

**Sensor parameters:**

| Parameter | Description |
|---|---|
| `name` | Name assigned to the sensor in ROS 2 Control |
| `label` | Name of the corresponding rigid body in Qualisys |

The `label` parameter must match the rigid body's name in Qualisys.

Multiple rigid bodies can be tracked by adding additional `<sensor>` blocks, each with its own name and label.

### 2. State Interfaces

For each configured rigid body, the hardware interface exports seven state interfaces:

| State interface | Description | Unit |
|---|---|---|
| `position.x` | X position | m |
| `position.y` | Y position | m |
| `position.z` | Z position | m |
| `orientation.x` | Quaternion X component | — |
| `orientation.y` | Quaternion Y component | — |
| `orientation.z` | Quaternion Z component | — |
| `orientation.w` | Quaternion W component | — |

For example, a sensor named `camera` exposes:

```text
camera/position.x
camera/position.y
camera/position.z
camera/orientation.x
camera/orientation.y
camera/orientation.z
camera/orientation.w
```

Position measurements are converted from millimeters to meters. Orientation is converted from the rotation matrix provided by Qualisys to a quaternion.

### 3. Pose Broadcaster

The sensor interfaces can be published using the standard ROS 2 Control `pose_broadcaster`.

Example controller configuration:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 200

    pose_broadcaster:
      type: pose_broadcaster/PoseBroadcaster

pose_broadcaster:
  ros__parameters:
    pose_name: "camera"
    frame_id: "camera"
    tf.enable: false
```

The `pose_name` parameter identifies the sensor whose state interfaces will be published.

For multiple rigid bodies, configure a separate pose broadcaster for each sensor, using distinct controller names and matching `pose_name` values.

**Note:** The standard `pose_broadcaster` replaces the previously used custom `pose_broadcaster_mod/PoseBroadcasterMod`. The custom parameters `sensor_to_control_freq_ratio` and `gpio_name` are therefore no longer required.

## Operation

The hardware interface follows the ROS 2 Control lifecycle:

1. **Initialization:** Reads the connection parameters and maps the configured sensor names to their Qualisys rigid-body labels.
2. **Activation:** Connects to the Qualisys server and retrieves the 6-DOF rigid-body settings.
3. **Reading:** Requests 6-DOF measurements and updates the corresponding state interfaces.
4. **Deactivation:** Stops streaming and disconnects from the Qualisys server.

Measurements are retrieved during the hardware interface's `read()` calls, which are executed by the ROS 2 controller manager.

Only rigid bodies whose labels match the configured sensors are processed.

### Invalid or Missing Measurements

All state interfaces are initialized to `NaN`.

When a rigid body's measurements contain invalid (`NaN`) values, the update is skipped and the previously stored state is retained.

Similarly, if a configured rigid body is absent from a received frame, its state interfaces are not updated.

**Important:** The current implementation does not provide an explicit measurement-validity flag or timestamp for each rigid body. Consequently, previously received measurements may remain available even when a rigid body is no longer tracked.

## Requirements

- ROS 2 with `ros2_control` and `ros2_controllers`.
- Qualisys QTM server accessible over the network.
- Qualisys RT Protocol SDK (provided through the underlying `mocap4qualisys` repository).
- At least one 6-DOF rigid body configured in Qualisys.

## Build and Launch

### 1. Build the package

From the ROS 2 workspace:

```bash
colcon build --packages-select qualisys_ros2_control_hw --symlink-install
source install/setup.bash
```

Ensure that the required ROS 2 dependencies and the Qualisys RT Protocol SDK are available before building.

### 2. Configure the hardware interface

Edit the following configuration files:

- `config/qualisys_sensor.config.xacro`: Configure the Qualisys server address, port, and rigid bodies to track.
- `config/robot_controllers.yaml`: Configure the controller manager and pose broadcaster.

The rigid-body labels defined in the Xacro file must match those configured in Qualisys QTM.

### 3. Launch the driver

Start the Qualisys QTM server and ensure that it is accessible over the network.

Launch the driver using:

```bash
ros2 launch qualisys_ros2_control_hw <launch_file>.launch.py
```

Replace `<launch_file>` with the name of the provided launch file.

The launch file automatically:

1. Generates the robot description from `config/qualisys_sensor.config.xacro`.
2. Starts the ROS 2 controller manager (`ros2_control_node`).
3. Loads the Qualisys hardware interface.
4. Spawns the `pose_broadcaster` controller.

### 3. Verify operation

List the available hardware interfaces:

```bash
ros2 control list_hardware_interfaces
```

For a sensor named `camera`, the seven pose state interfaces should be available.

Check that the broadcaster is active:

```bash
ros2 control list_controllers
```

Inspect the published topics:

```bash
ros2 topic list
```

## License

Apache License 2.0.

Copyright 2026, ICube Laboratory, University of Strasbourg, France.

Author: Mahdi Chaari