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

import romea_common_description
from romea_mobile_base_description import (
    get_complete_configuration,
    get_specification_units,
)

import xacro
import yaml


def get_specifications_path_file():
    return get_package_share_directory("robucar_description") + "/config/robucar.yaml"


def get_specifications_configuration():
    with open(get_specifications_path_file(), "r") as f:
        return yaml.safe_load(f)


def get_configuration():
    specifications = get_specifications_configuration()
    configuration = get_complete_configuration(specifications)
    configuration["model"] = "robucar"
    configuration["version"] = ""
    configuration["manufacturer"] = "robosoft"
    return configuration


def generate_configuration_file(configuration, extended):
    units = get_specification_units()
    return romea_common_description.generate_configuration_file(configuration, units, extended)


def generate_ros2_control_description(prefix, mode, base_name):
    if mode == "simulation":
        mode += "_gazebo_classic"

    ros2_control_xacro_file = (
        get_package_share_directory("robucar_description")
        + "/ros2_control/robucar.ros2_control.urdf.xacro"
    )

    ros2_control_urdf_xml = xacro.process_file(
        ros2_control_xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "base_name": base_name,
        },
    )

    return ros2_control_urdf_xml.toprettyxml(indent="  ")


def generate_urdf_description(
    prefix, mode, base_name, controller_manager_config_yaml_file, ros_prefix
):
    if mode == "simulation":
        mode += "_gazebo_classic"

    xacro_file = (
        get_package_share_directory("robucar_description")
        + "/urdf/robucar.urdf.xacro"
    )

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "base_name": base_name,
            "controller_manager_config_yaml_file": controller_manager_config_yaml_file,
            "ros_prefix": ros_prefix,
        },
    )

    return urdf_xml.toprettyxml(indent="  ")


def urdf(prefix, mode, base_name, controller_manager_config_yaml_file, ros_prefix):
    return generate_urdf_description(
        prefix, mode, base_name, controller_manager_config_yaml_file, ros_prefix
    )
