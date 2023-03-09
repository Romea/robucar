# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license


# import pytest
import xml.etree.ElementTree as ET
from robucar_description import urdf


def urdf_xml(mode):
    prefix = "robot_"
    ros_namespace = "/robot"
    controller_conf_yaml_file = mode + "_controller.yaml"
    return ET.fromstring(urdf(prefix, mode, controller_conf_yaml_file, ros_namespace))


def test_footprint_link_name():
    assert urdf_xml("live").find("link").get("name") == "robot_base_footprint"


def test_hardware_plugin_name():

    assert urdf_xml("live").find(
        "ros2_control/hardware/plugin"
    ).text == "robucar_hardware/RobucarHardware"

    assert urdf_xml("simulation").find(
        "ros2_control/hardware/plugin"
    ).text == "romea_mobile_base_gazebo/GazeboSystemInterface2AS4WD"


def test_controller_filename_name():
    assert (
        urdf_xml("simulation").find("gazebo/plugin/parameters").text
        == "simulation_controller.yaml"
    )
