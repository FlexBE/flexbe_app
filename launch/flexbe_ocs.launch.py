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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals


def generate_launch_description():

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
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="False")

    flexbe_app = Node(name="flexbe_app", package="flexbe_app", executable="run_app", output="screen",
                      arguments=[{LaunchConfiguration("offline_arg")}])

    behavior_mirror = Node(name="behavior_mirror", package="flexbe_mirror",
                           executable="behavior_mirror_sm",
                           condition=LaunchConfigurationNotEquals("offline_arg", "--offline"))

    behavior_launcher = Node(name="behavior_launcher", package="flexbe_widget",
                             executable="be_launcher", output="screen",
                             condition=LaunchConfigurationNotEquals("offline_arg", "--offline"))

    return LaunchDescription([
        offline,
        offline_arg,
        online_arg,
        no_app,
        use_sim_time,
        behavior_mirror,
        flexbe_app,
        behavior_launcher
    ])
