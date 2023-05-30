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


import xacro

from ament_index_python.packages import get_package_share_directory


def urdf(prefix, mode, base_name, controller_manager_config_yaml_file, ros_prefix):

    ros2_control_xacro_file = (
        get_package_share_directory("robucar_description")
        + "/urdf/ros2_control/robucar.ros2_control.urdf.xacro"
    )

    ros2_control_urdf_xml = xacro.process_file(
        ros2_control_xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "base_name": base_name,
        },
    )

    ros2_control_config_urdf_file = "/tmp/"+prefix+base_name+"_ros2_control.urdf"

    with open(ros2_control_config_urdf_file, "w") as f:
        f.write(ros2_control_urdf_xml.toprettyxml())

    xacro_file = (
        get_package_share_directory("robucar_description") + "/urdf/robucar.urdf.xacro"
    )

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "base_name": base_name,
            "controller_manager_config_yaml_file": controller_manager_config_yaml_file,
            "ros2_control_config_urdf_file": ros2_control_config_urdf_file,
            "ros_prefix": ros_prefix
        },
    )

    return urdf_xml.toprettyxml()
