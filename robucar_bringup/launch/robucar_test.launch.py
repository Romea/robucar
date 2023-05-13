# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    joystick_type = LaunchConfiguration("joystick_type").perform(context)
    joystick_device = LaunchConfiguration("joystick_device").perform(context)

    actions = []

    if mode == "simulation":

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("gazebo_ros"),
                                "launch",
                                "gazebo.launch.py",
                            ]
                        )
                    ]
                )
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("robucar_bringup"),
                            "launch",
                            "robucar_base.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={"mode": mode}.items(),
        )
    )

    actions.append(PushRosNamespace("robufast"))

    teleop_configuration_file_path = (
        get_package_share_directory("robucar_description") + "/config/teleop.yaml"
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("romea_teleop_drivers")
                + "/launch/teleop.launch.py"
            ),
            launch_arguments={
                "robot_type": "robucar",
                "joystick_type": joystick_type,
                "joystick_driver": "joy",
                "joystick_topic": "/robufast/joystick/joy",
                "teleop_configuration_file_path": teleop_configuration_file_path,
            }.items(),
        )
    )

    actions.append(PushRosNamespace("joystick"))

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("romea_joystick_bringup"),
                            "launch",
                            "drivers/joy.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "device": joystick_device,
                "dead_zone": "0.05",
                "autorepeat_rate": "10.0",
                "frame_id": "joy",
            }.items(),
        )
    )

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode", default_value="simulation"))

    declared_arguments.append(
        DeclareLaunchArgument("joystick_type", default_value="xbox")
    )

    declared_arguments.append(
        DeclareLaunchArgument("joystick_device", default_value="/dev/input/js0")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
