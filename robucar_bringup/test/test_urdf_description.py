# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

import subprocess

from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory

import xml.etree.ElementTree as ET


def urdf_xml(mode):

    exe = (
        get_package_prefix("robucar_bringup") + "/lib/robucar_bringup/urdf_description.py"
    )

    return ET.fromstring(
        subprocess.check_output(
            [exe, "mode:" + mode, "robot_namespace:robot"],
            encoding="utf-8",
        )
    )


def test_footprint_link_name():
    assert urdf_xml("live").find("link").get("name") == "robot_base_footprint"


def test_hardware_plugin_name():

    assert urdf_xml("live").find(
        "ros2_control/hardware/plugin"
    ).text == "aroco_hardware/RobufastHardware"

    assert urdf_xml("simulation").find(
        "ros2_control/hardware/plugin"
    ).text == "romea_mobile_base_gazebo/GazeboSystemInterface2AS4WD"


def test_controller_filename_name():
    assert (
        urdf_xml("simulation").find("gazebo/plugin/parameters").text
        == get_package_share_directory("aroco_bringup")
        + "/config/controller_manager.yaml"
    )
