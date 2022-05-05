from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    info_file = get_package_share_directory("robucar_description") + "/config/robucar.yaml"
    urdf_file = get_package_share_directory("robucar_description") + "/urdf/robucar.urdf.xacro"
    controller_manager_yaml_file = get_package_share_directory("robucar_bringup") + "/config/controller_manager.yaml"
    mobile_base_controller_yaml_file = get_package_share_directory("robucar_bringup") + "/config/mobile_base_controller.yaml"

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_file,
            " mode:=live",
            " controller_conf_yaml_file:=",
            controller_manager_yaml_file,
        ]
    )

    robot_description = {"robot_description": robot_description_content}


    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,controller_manager_yaml_file],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
#         arguments=['--ros-args', '--log-level', 'debug']
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    mobile_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="mobile_base_controller_spawner",
        arguments=["mobile_base_controller","--param-file",mobile_base_controller_yaml_file],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster,
            mobile_base_controller,
        ]
    )
