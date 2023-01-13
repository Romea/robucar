# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

import xacro

from ament_index_python.packages import get_package_share_directory


def urdf(prefix, mode, controller_conf_yaml_file):

    xacro_file = (
        get_package_share_directory("robucar_description")
        + "/urdf/robucar.urdf.xacro"
    )

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "controller_conf_yaml_file": controller_conf_yaml_file,
        },
    )

    return urdf_xml.toprettyxml()
