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
from launch.conditions import (
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    urdf_description = LaunchConfiguration("urdf_description").perform(context)

    if robot_namespace:
        robot_description_name = "/" + robot_namespace + "/robot_description"
        controller_manager_name = "/" + robot_namespace + "/base/controller_manager"
        joints_prefix = robot_namespace + "_"
    else:
        robot_description_name = "/robot_description"
        controller_manager_name = "/base/controller_manager"
        joints_prefix = ""

    use_sim_time = (mode == "simulation") or (mode == "replay")

    base_description_yaml_file = (
        get_package_share_directory("robucar_description") + "/config/robucar.yaml"
    )

    controller_manager_yaml_file = (
        get_package_share_directory("robucar_bringup")
        + "/config/controller_manager.yaml"
    )

    base_controller_yaml_file = (
        get_package_share_directory("robucar_bringup")
        + "/config/mobile_base_controller.yaml"
    )

    robot_description = {"robot_description": urdf_description}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output={
            'stdout': 'log',
            'stderr': 'log',
        },
    )

    spawn_entity = Node(
        condition=LaunchConfigurationEquals("mode", "simulation"),
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            robot_description_name,
            "-entity",
            robot_namespace,
        ],
        output={
            'stdout': 'log',
            'stderr': 'log',
        },
    )

    controller_manager = Node(
        condition=LaunchConfigurationEquals("mode", "live"),
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_manager_yaml_file],
        namespace="base",
        # output="screen",
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_mobile_base_controllers"),
                        "launch",
                        "mobile_base_controller.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "joints_prefix": joints_prefix,
            "controller_name": "mobile_base_controller",
            "controller_manager_name": controller_manager_name,
            "base_description_yaml_filename": base_description_yaml_file,
            "base_controller_yaml_filename": base_controller_yaml_file,
        }.items(),
        condition=LaunchConfigurationNotEquals("mode", "replay"),
    )

    cmd_mux = Node(
        condition=LaunchConfigurationNotEquals("mode", "replay"),
        package="romea_cmd_mux",
        executable="cmd_mux_node",
        name="cmd_mux",
        parameters=[{"topics_type": "romea_mobile_base_msgs/TwoAxleSteeringCommand"}],
        remappings=[("~/out", "controller/cmd_two_axle_steering")],
        namespace="base",
        output="screen",
    )

    return [
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=use_sim_time),
                PushRosNamespace(robot_namespace),
                robot_state_publisher,
                spawn_entity,
                controller_manager,
                controller,
                # teleop,
                cmd_mux,
            ]
        )
    ]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode", default_value="simulation"))

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace", default_value="robufast")
    )

    urdf_description = Command(
        [
            ExecutableInPackage("urdf_description.py", "robucar_bringup"),
            " robot_namespace:",
            LaunchConfiguration("robot_namespace"),
            " mode:",
            LaunchConfiguration("mode"),
        ]
    )

    declared_arguments.append(
        DeclareLaunchArgument("urdf_description", default_value=urdf_description)
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
