# robucar

## Overview

`robucar` groups the ROS2 packages that describe, launch and control the Robucar mobile base, also called Robufast, in live and simulation modes.

This repository-level README gives a map of the stack. Detailed information about each package can be found in the corresponding package README.

## Packages

| Package | Role |
| --- | --- |
| `robucar` | Metapackage that groups the Robucar ROS2 packages. |
| `robucar_description` | Robot-specific description layer for Robucar, including configuration files, URDF/Xacro descriptions, meshes and ros2_control descriptions. |
| `robucar_bringup` | Main integration entry point for generating Robucar configuration files, URDF descriptions, ros2_control descriptions and launch files. |
| `robucar_hardware` | Live `ros2_control` hardware plugin for the Robucar mobile base, built on the generic `2AS4WD` hardware abstraction. |

## Usage

In most cases, start with `robucar_bringup`. It is the user-facing entry point of the stack and the package used by `romea_mobile_base_meta_bringup` when a Robucar model is selected from a mobile base meta-description.

The Robucar stack is a robot-specific specialization of `romea_mobile_base`. The mobile base architecture is `2AS4WD`; `robucar_description` provides the concrete geometry and generated descriptions, `robucar_hardware` provides the live hardware implementation, and `robucar_bringup` connects these pieces to the generic mobile base launch workflow.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

The `robucar` project was developed by Jean Laneurit in the context of the BAUDET ROB ANR project.

## Contact

For questions or comments about this project, please contact [Jean Laneurit](mailto:jean.laneurit@inrae.fr).
