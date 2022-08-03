from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals


offline = DeclareLaunchArgument("offline",
                                description="Treat FlexBE App as offline editor () Editor mode as default",
                                default_value="false")

# Change the default value based on passing a true/false string to offline, or allow setting directly
offline_arg = DeclareLaunchArgument("offline_arg",
                                    description="Optionally specify FlexBE App offline Editor mode ('--offline')",
                                    default_value="--offline",
                                    condition=LaunchConfigurationEquals("offline", "true"))
online_arg = DeclareLaunchArgument("offline_arg",
                                    description="Optionally specify FlexBE App offline Editor mode ('--offline') default=''",
                                    default_value="",
                                    condition=LaunchConfigurationEquals("offline", "false"))

no_app = DeclareLaunchArgument("no_app", default_value="false")

flexbe_app = Node(name="flexbe_app", package="flexbe_app", executable="run_app", output="screen",
                  arguments=[{LaunchConfiguration("offline_arg")}])

behavior_mirror = Node(name="behavior_mirror", package="flexbe_mirror",
                       executable="behavior_mirror_sm",
                       condition=LaunchConfigurationNotEquals("offline_arg", "--offline"))

behavior_launcher = Node(name="behavior_launcher", package="flexbe_widget",
                         executable="be_launcher", output="screen",
                         condition=LaunchConfigurationNotEquals("offline_arg", "--offline"))

def generate_launch_description():
    return LaunchDescription([
            offline,
            offline_arg,
            online_arg,
            no_app,
            behavior_mirror,
            flexbe_app,
            behavior_launcher
        ])
