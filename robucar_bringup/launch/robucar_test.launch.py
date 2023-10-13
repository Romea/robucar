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

from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    joystick_type = LaunchConfiguration("joystick_type").perform(context)
    joystick_device = LaunchConfiguration("joystick_device").perform(context)
    robot_urdf_description = LaunchConfiguration("robot_urdf_description").perform(context)

    robot = []

    if mode == "simulation":

        world = PathJoinSubstitution(
            [
                FindPackageShare("romea_simulation_gazebo_worlds"),
                "worlds",
                "friction_cone.world",
            ]
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("gazebo_ros"),
                                "launch",
                                "gzserver.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={"world": world, "verbose": "false"}.items(),
            )
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("gazebo_ros"),
                                "launch",
                                "gzclient.launch.py",
                            ]
                        )
                    ]
                )
            )
        )

        robot_description_file = "/tmp/robufast_description.urdf"
        with open(robot_description_file, "w") as f:
            f.write(robot_urdf_description)

        robot.append(
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-file",
                    robot_description_file,
                    "-entity",
                    "robufast",
                ],
                output={
                    'stdout': 'log',
                    'stderr': 'log',
                },
            )
        )

    robot.append(
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

    teleop_configuration_file_path = (
        get_package_share_directory("robucar_description") + "/config/teleop.yaml"
    )

    robot.append(
        GroupAction(
            actions=[
                PushRosNamespace("robufast"),
                PushRosNamespace("base"),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        get_package_share_directory("robucar_bringup")
                        + "/launch/robucar_teleop.launch.py"
                    ),
                    launch_arguments={
                        "joystick_type": joystick_type,
                        "joystick_driver": "joy",
                        "joystick_topic": "/robufast/joystick/joy",
                        "teleop_configuration_file_path": teleop_configuration_file_path,
                    }.items(),
                )
            ]
        )
    )

    robot.append(
        GroupAction(
            actions=[
                PushRosNamespace("robufast"),
                PushRosNamespace("joystick"),

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
            ]
        )
    )

    return robot


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode", default_value="simulation"))

    urdf_description = Command(
        [
            ExecutableInPackage("urdf_description.py", "robucar_bringup"),
            " robot_namespace:robufast",
            " base_name:base",
            " mode:",
            LaunchConfiguration("mode"),
        ],
        on_stderr="ignore",
    )

    declared_arguments.append(
        DeclareLaunchArgument("robot_urdf_description", default_value=urdf_description)
    )

    declared_arguments.append(
        DeclareLaunchArgument("joystick_type", default_value="xbox")
    )

    declared_arguments.append(
        DeclareLaunchArgument("joystick_device", default_value="/dev/input/js0")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
