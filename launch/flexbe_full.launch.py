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

        DeclareLaunchArgument("log_enabled", default_value="False"),
        DeclareLaunchArgument("log_folder", default_value="~/.flexbe_logs"),
        DeclareLaunchArgument("log_serialize", default_value="yaml"),
        DeclareLaunchArgument("log_level", default_value="INFO"),
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("enable_clear_imports", default_value="False",
                                description="Delete behavior-specific module imports after execution."),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(flexbe_onboard_dir + "/behavior_onboard.launch.py"),
            launch_arguments={
                "log_enabled": LaunchConfiguration("log_enabled"),
                "log_folder": LaunchConfiguration("log_folder"),
                "log_serialize": LaunchConfiguration("log_serialize"),
                "log_level": LaunchConfiguration("log_level"),
                "enable_clear_imports": LaunchConfiguration("enable_clear_imports"),
                "use_sim_time": LaunchConfiguration("use_sim_time") 
            }.items()
        )
    ])
