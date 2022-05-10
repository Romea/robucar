from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={"verbose": "false"}.items(),
    )

    info_file = get_package_share_directory("robucar_description") + "/config/robucar.yaml"
    urdf_file = get_package_share_directory("robucar_description") + "/urdf/robucar.urdf.xacro"
    controller_manager_yaml_file = get_package_share_directory("robucar_bringup") + "/config/controller_manager.yaml"

    print(controller_manager_yaml_file)
    mobile_base_controller_yaml_file = get_package_share_directory("aroco_bringup") + "/config/mobile_base_controller.yaml"

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robucar_description"),
                    "urdf",
                    "robucar.urdf.xacro",
                ]
            ),
            " use_sim:=true ",
            "controller_conf_yaml_file:=",
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

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "robucar"],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner.py",
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
            gazebo,
            robot_state_publisher,
            spawn_entity,
            joint_state_broadcaster,
            mobile_base_controller,
#            spawn_controller,
        ]
    )
