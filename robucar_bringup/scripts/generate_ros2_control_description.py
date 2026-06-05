#!/usr/bin/env python3

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

import sys

from robucar_bringup import generate_ros2_control_description
from romea_common_meta_bringup.utils import complete_mode, robot_urdf_prefix


if __name__ == "__main__":
    parameters = {}
    for argument in sys.argv[1:]:
        name, value = argument.split(":")
        parameters[name] = value

    mode = complete_mode(parameters["mode"])
    base_name = parameters["base_name"]
    prefix = robot_urdf_prefix(parameters["robot_namespace"])
    print(generate_ros2_control_description(prefix, mode, base_name))
