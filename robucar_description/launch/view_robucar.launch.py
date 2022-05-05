from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    romea_mobile_base_description_package_prefix = get_package_share_directory('romea_mobile_base_description')
    urdf_file = get_package_share_directory("robucar_description") + "/urdf/robucar.urdf.xacro"

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([romea_mobile_base_description_package_prefix,'/launch/view_robot.launch.py']),
            launch_arguments = {'urdf_file': urdf_file}.items(),
        ),
    ])
