from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


offline = DeclareLaunchArgument("offline", default_value="false")

offline_arg = None

if LaunchConfiguration("offline_arg"):
    offline_arg = DeclareLaunchArgument("offline_arg", default_value="--offline")
else:
    offline_arg = DeclareLaunchArgument("offline_arg", default_value="")

no_app = DeclareLaunchArgument("no_app", default_value="false")

behavior_mirror = Node(name="behavior_mirror", package="flexbe_mirror", executable="behavior_mirror_sm")

flexbe_app = Node(name="flexbe_app", package="flexbe_app", executable="run_app", output="screen",
                arguments=[{LaunchConfiguration("offline_arg")}])

behavior_launcher = Node(name="behavior_launcher", package="flexbe_widget", executable="be_launcher", output="screen")

def generate_launch_description():
    return LaunchDescription([
            offline,
            offline_arg,
            no_app,
            behavior_mirror,
            flexbe_app,
            behavior_launcher
        ])
