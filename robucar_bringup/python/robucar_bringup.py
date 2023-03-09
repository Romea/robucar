# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license


from ament_index_python.packages import get_package_share_directory
import robucar_description


def urdf_description(prefix, mode, ros_namespace):

    controller_manager_yaml_file = (
        get_package_share_directory("robucar_bringup")
        + "/config/controller_manager.yaml"
    )

    return robucar_description.urdf(prefix, mode, controller_manager_yaml_file, ros_namespace)
