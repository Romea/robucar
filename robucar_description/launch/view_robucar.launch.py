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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    romea_mobile_base_description_package_prefix = get_package_share_directory(
        "romea_mobile_base_description"
    )
    urdf_file = (
        get_package_share_directory("robucar_description") + "/urdf/robucar.urdf.xacro"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        romea_mobile_base_description_package_prefix,
                        "/launch/view_robot.launch.py",
                    ]
                ),
                launch_arguments={"urdf_file": urdf_file}.items(),
            )
        ],
        on_stderr="ignore",
    )
