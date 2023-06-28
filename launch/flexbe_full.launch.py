# Copyright 2023 Philipp Schillinger, Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Philipp Schillinger,
#      Christopher Newport University nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    flexbe_app_dir = get_package_share_directory('flexbe_app')
    flexbe_onboard_dir = get_package_share_directory('flexbe_onboard')

    return LaunchDescription([
        DeclareLaunchArgument("no_app", default_value="false"),
        DeclareLaunchArgument("log_enabled", default_value="False"),
        DeclareLaunchArgument("log_folder", default_value="~/.flexbe_logs"),
        DeclareLaunchArgument("log_serialize", default_value="yaml"),
        DeclareLaunchArgument("log_level", default_value="INFO"),
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("enable_clear_imports", default_value="False",
                              description="Delete behavior-specific module imports after execution."),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(flexbe_app_dir + "/launch/flexbe_ocs.launch.py"),
            launch_arguments={
                "offline": "false",
                "no_app": LaunchConfiguration("no_app"),
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }.items()
        ),

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
