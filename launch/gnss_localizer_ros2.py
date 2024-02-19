import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    # Define arguments for launch files
    ns = LaunchConfiguration("ns") 

    declare_ns = DeclareLaunchArgument(
        "ns",
        default_value="ic120",
        description=""
    )


    # Node
    gnss_localizer_ros2 = Node(
        package="gnss_localizer_ros2",
        name="gnss_localizer_ros2",
        namespace=ns,
        executable="gnss_localizer_ros2",
        output="screen",
    )


    ld = LaunchDescription()
    ld.add_action(declare_ns)
    ld.add_action(gnss_localizer_ros2)

    return ld
