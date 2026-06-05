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

from ament_index_python.packages import get_package_share_directory
from robucar_description import get_specifications_path_file

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

import romea_common_meta_bringup.ros_launch as common
import romea_joystick_meta_bringup.ros_launch as joystick


def launch_setup(context, *args, **kwargs):
    mode = common.get_mode(context)
    joystick_topic = joystick.get_joystick_topic(context)
    joystick_configuration_file_path = joystick.get_joystick_configuration_file_path(context)
    mobile_base_configuration_file_path = get_specifications_path_file()
    teleop_configuration_file_path = LaunchConfiguration(
        "teleop_configuration_file_path"
    ).perform(context)

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_mobile_base_teleop") + "/launch/teleop.launch.py"
        ),
        launch_arguments={
            "mobile_base_configuration_file_path": mobile_base_configuration_file_path,
            "joystick_configuration_file_path": joystick_configuration_file_path,
            "teleop_configuration_file_path": teleop_configuration_file_path,
            "joystick_topic": joystick_topic,
        }.items(),
    )

    return [
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=(mode != "live")),
                teleop,
            ]
        )
    ]


def generate_launch_description():
    default_teleop_configuration_file_path = (
        get_package_share_directory("robucar_description") + "/config/teleop.yaml"
    )

    return LaunchDescription(
        [
            common.declare_mode(),
            joystick.declare_joystick_topic(),
            joystick.declare_joystick_configuration_file_path(),
            DeclareLaunchArgument(
                "teleop_configuration_file_path",
                default_value=default_teleop_configuration_file_path,
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
