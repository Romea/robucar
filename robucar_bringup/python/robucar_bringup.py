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
import robucar_description


def get_configuration():
    return robucar_description.get_configuration()


def generate_configuration_file(extended):
    configuration = get_configuration()
    return robucar_description.generate_configuration_file(configuration, extended)


def generate_ros2_control_description(prefix, mode, base_name):
    return robucar_description.generate_ros2_control_description(
        prefix, mode, base_name
    )


def generate_urdf_description(prefix, mode, base_name, ros_prefix):
    controller_manager_yaml_file = (
        get_package_share_directory("robucar_bringup")
        + "/config/controller_manager.yaml"
    )

    return robucar_description.generate_urdf_description(
        prefix, mode, base_name, controller_manager_yaml_file, ros_prefix
    )


def urdf_description(prefix, mode, base_name, ros_prefix):
    return generate_urdf_description(prefix, mode, base_name, ros_prefix)
