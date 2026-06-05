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

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch_ros.actions import Node, SetParameter

import romea_common_meta_bringup.ros_launch as common
import romea_common_meta_bringup.utils as utils
import romea_mobile_base_meta_bringup.ros_launch as mobile_base


def launch_setup(context, *args, **kwargs):
    mode = common.get_mode(context)
    if "replay" in mode:
        return []

    robot_namespace = common.get_robot_namespace(context)
    robot_urdf_description = common.get_robot_urdf_description(context)
    robot_ros2_control_description = common.get_robot_ros2_control_description(context)

    base_configuration_file_path = (
        f'{get_package_share_directory("robucar_description")}/config/robucar.yaml'
    )

    controller_manager_configuration_file_path = (
        f'{get_package_share_directory("robucar_bringup")}/config/controller_manager.yaml'
    )

    base_controller_configuration_file_path = (
        f'{get_package_share_directory("robucar_bringup")}/config/mobile_base_controller.yaml'
    )

    ros2_control_description_node = Node(
        package="romea_common_meta_bringup",
        executable="urdf_broadcaster_node",
        name="ros2_control_description",
        parameters=[
            {
                "robot_description": utils.complete_robot_description(
                    robot_urdf_description, [robot_ros2_control_description]
                )
            }
        ],
    )

    controller_manager = Node(
        condition=IfCondition(PythonExpression(["'gazebo' not in '", mode, "'"])),
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_manager_configuration_file_path],
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_mobile_base_controllers")
            + "/launch/mobile_base_controller.launch.py"
        ),
        launch_arguments={
            "joints_prefix": utils.robot_urdf_prefix(robot_namespace),
            "controller_name": "mobile_base_controller",
            "base_configuration_file_path": base_configuration_file_path,
            "base_controller_configuration_file_path": base_controller_configuration_file_path,
        }.items(),
    )

    cmd_mux = Node(
        package="romea_cmd_mux",
        executable="cmd_mux_node",
        name="cmd_mux",
        parameters=[{"topics_type": "romea_mobile_base_msgs/TwoAxleSteeringCommand"}],
        remappings=[("~/out", "controller/cmd_two_axle_steering")],
        output="screen",
    )

    return [
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=(mode != "live")),
                ros2_control_description_node,
                controller_manager,
                controller,
                cmd_mux,
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            common.declare_mode(),
            common.declare_robot_namespace(),
            mobile_base.declare_base_name("base"),
            common.declare_robot_urdf_description(
                common.generate_robot_urdf_description("robucar_bringup")
            ),
            common.declare_robot_ros2_control_description(
                common.generate_robot_ros2_control_description("robucar_bringup")
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
