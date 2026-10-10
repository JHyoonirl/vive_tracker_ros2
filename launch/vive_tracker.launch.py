from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_numbers", description="Python-style list of Tracker serials"),
        DeclareLaunchArgument("tracker_ids", default_value="['tracker']"),
        DeclareLaunchArgument("namespaces", default_value="['vive']"),
        Node(
            package="vive_tracker_ros",
            executable="vive_tracker_node",
            name="vive_tracker_node",
            parameters=[{
                "serial_numbers": PythonExpression([LaunchConfiguration("serial_numbers")]),
                "tracker_ids": PythonExpression([LaunchConfiguration("tracker_ids")]),
                "namespaces": PythonExpression([LaunchConfiguration("namespaces")]),
            }],
            output="screen",
        ),
    ])
