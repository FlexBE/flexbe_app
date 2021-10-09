from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

flexbe_app_dir = get_package_share_directory('flexbe_app')
flexbe_onboard_dir = get_package_share_directory('flexbe_onboard')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "no_app",
            default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(flexbe_app_dir + "/launch/flexbe_ocs.launch.py"),
            launch_arguments={
                "offline": "false",
                "no_app": LaunchConfiguration("no_app")
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(flexbe_onboard_dir + "/behavior_onboard.launch.py")
        )
        ])
