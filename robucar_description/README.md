# robucar_description

## 1) Overview

`robucar_description` provides the robot-specific description layer for the Robucar mobile base.

It extends `romea_mobile_base_description` with the concrete configuration, URDF/Xacro files, meshes and `ros2_control` descriptions required to instantiate the Robucar robot.

The Robucar mobile base uses:

| Configuration file | Mobile base architecture | Command type |
|---|---|---|
| `config/robucar.yaml` | `2AS4WD` | `two_axle_steering` |

This architecture has front and rear axle steering and four driving wheels. It is controlled as a two-axle-steering mobile base.

## 2) Robot configuration

The `config/` directory contains:

| File | Purpose |
|---|---|
| `robucar.yaml` | full Robucar mobile base configuration |
| `teleop.yaml` | default two-axle-steering teleoperation configuration |

The robot configuration follows the structure defined by `romea_mobile_base_description` and contains:

* the mobile base architecture (`2AS4WD`);
* geometry, wheel dimensions and chassis bounding box;
* front and rear axle steering command and feedback information;
* wheel speed command and feedback information;
* inertia and control point;
* link and joint names used in the URDF and `ros2_control` descriptions.

## 3) URDF and ros2_control descriptions

The URDF description is built from:

| Path | Role |
|---|---|
| `urdf/robucar.urdf.xacro` | main URDF entry point |
| `urdf/robucar.xacro` | Robucar mobile base macro |
| `urdf/robucar.simulation.xacro` | simulator-specific Gazebo or Gazebo Classic plugin insertion |
| `urdf/visual/` | visual Xacro fragments for chassis, wheels and arms |
| `meshes/` | Robucar chassis and wheel meshes |

The Robucar macro reuses the `base2ASxxx.chassis.xacro` template from `romea_mobile_base_description` and specializes it with Robucar geometry, link names, joint names and visual meshes.

The `ros2_control` description is built from:

| Path | Role |
|---|---|
| `ros2_control/robucar.ros2_control.urdf.xacro` | main `ros2_control` entry point |
| `ros2_control/robucar.ros2_control.xacro` | Robucar `ros2_control` macro |

Depending on the selected mode, the `ros2_control` description selects:

| Mode | Hardware plugin |
|---|---|
| `live` | `robucar_hardware/RobucarHardware` |
| `simulation`, `simulation_gazebo_classic` | `romea_mobile_base_gazebo/GazeboSystemInterface2AS4WD` |
| `simulation_gazebo` | `romea_mobile_base_gazebo/GazeboSystemInterface2AS4WD` |

## 4) Python API

The installed Python module provides helper functions used by `robucar_bringup` and by the meta-bringup workflow.

| Function | Purpose |
|---|---|
| `get_specifications_path_file()` | returns the Robucar configuration file path |
| `get_specifications_configuration()` | loads the full robot configuration |
| `get_configuration()` | returns the compact mobile base configuration completed with manufacturer, model and version |
| `generate_configuration_file(configuration, extended)` | serializes the compact configuration |
| `generate_urdf_description(...)` | generates the Robucar URDF description |
| `generate_ros2_control_description(...)` | generates the Robucar `ros2_control` description |
| `urdf(...)` | compatibility wrapper around `generate_urdf_description(...)` |

Example:

```python
from robucar_description import get_configuration

configuration = get_configuration()
```

## 5) Relation with other packages

`robucar_description` is the Robucar specialization of `romea_mobile_base_description`:

* `romea_mobile_base_description` provides the generic `2AS4WD` description and `ros2_control` templates;
* `robucar_description` provides the Robucar configuration, meshes and Xacro specialization;
* `robucar_hardware` provides the live `ros2_control` hardware plugin;
* `robucar_bringup` uses this package to generate the URDF and `ros2_control` artifacts for live and simulation modes.
