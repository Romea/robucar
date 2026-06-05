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
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import romea_common_meta_bringup.ros_launch as common
import romea_mobile_base_meta_bringup.ros_launch as mobile_base


def launch_setup(context, *args, **kwargs):
    mode = common.get_mode(context)
    robot_namespace = common.get_robot_namespace(context)
    robot_urdf_description = common.get_robot_urdf_description(context)

    robot = []

    if mode == "simulation_gazebo_classic":
        world = (
            get_package_share_directory("romea_simulation_gazebo_worlds")
            + "/worlds/friction_cone.world"
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("gazebo_ros")
                    + "/launch/gzserver.launch.py"
                ),
                launch_arguments={"world": world, "verbose": "false"}.items(),
            )
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("gazebo_ros")
                    + "/launch/gzclient.launch.py"
                )
            )
        )

        robot_description_file = "/tmp/robucar_description.urdf"
        with open(robot_description_file, "w") as f:
            f.write(robot_urdf_description)

        robot.append(
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                exec_name="gazebo_spawn_entity",
                arguments=["-file", robot_description_file, "-entity", robot_namespace],
                output={"stdout": "log", "stderr": "log"},
            )
        )

    else:
        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("ros_gz_sim")
                    + "/launch/gz_sim.launch.py"
                ),
                launch_arguments={
                    "gz_args": "-g",
                    "on_exit_shutdown": "True",
                }.items(),
            )
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("ros_gz_sim")
                    + "/launch/gz_server.launch.py"
                ),
                launch_arguments={
                    "world_sdf_file": "empty.sdf",
                    "world_sdf_string": "world",
                }.items(),
            )
        )

        robot_description_file = "/tmp/robucar_description.urdf"
        with open(robot_description_file, "w") as f:
            f.write(robot_urdf_description)

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("ros_gz_sim")
                    + "/launch/gz_spawn_model.launch.py"
                ),
                launch_arguments=[
                    ("file", robot_description_file),
                    ("entity_name", robot_namespace),
                ],
            )
        )

        robot.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                output="screen",
            )
        )

    return robot


def generate_launch_description():
    return LaunchDescription(
        [
            common.declare_mode("simulation"),
            common.declare_robot_namespace("robufast"),
            mobile_base.declare_base_name("base"),
            common.declare_robot_urdf_description(
                common.generate_robot_urdf_description("robucar_bringup")
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
