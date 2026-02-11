from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='qualisys_sensor.config.xacro',
            description='URDF/XACRO description file with the axis.',
        )
    )

    description_file = LaunchConfiguration('description_file')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("qualisys_ros2_control_hw"),
                    "config", 
                    description_file,
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("qualisys_ros2_control_hw"),
            "config",
            "robot_controllers.yaml",
        ]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    pose_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pose_broadcaster", "-c", "/controller_manager"],
    )

    nodes = [
        control_node,
        pose_broadcaster_spawner
    ]

    return LaunchDescription(
        declared_arguments + 
        nodes)