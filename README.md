# [ground_vehicle_kinematics](https://github.com/jfrascon/ground_vehicle_kinematics)

This package provides utilities and ROS 2 nodes to compute forward and inverse kinematics for ground vehicles. It is intended as a generic foundation for different wheel-based vehicle models, and currently includes a documented implementation for a three-swerve platform.

## Main Features

- Implements forward and inverse kinematics algorithms for ground vehicles.
- Includes ready-to-use ROS 2 nodes written in C++.
- Integrates with other ROS 2 packages using standard messages (`geometry_msgs`, `sensor_msgs`).
- Uses Eigen3 for efficient mathematical computations.
- Provides configuration files and launch scripts for easy setup and testing.
- Is structured to support multiple ground-vehicle kinematic models.
- Currently includes technical documentation and an implementation example for a three-swerve configuration.

## Package Structure

- [`config/`](config/): Example YAML configuration files.
- [`doc/`](doc/): Technical documentation, derivations, and reference material.
- [`include/`](include/): Public headers for types, solvers, and ROS-facing interfaces.
- [`launch/`](launch/): Launch files to start nodes and configurations.
- [`src/`](src/): Main source code for solvers and ROS nodes.

## Typical Usage

This package serves as a foundation for controlling and simulating the kinematics of mobile robots, enabling the conversion of velocity commands into wheel movements and vice versa, and facilitating integration with navigation and control systems for ground vehicles.

## Current Example: Three-Swerve

- The currently documented model is a three-swerve configuration. Its kinematic conventions and derivations are available in [`doc/kinematics_en.pdf`](doc/kinematics_en.pdf) and [`doc/kinematics_es.pdf`](doc/kinematics_es.pdf).
- The direct kinematics of the current three-swerve implementation are formulated as a least-squares problem `A x = b` solved with a precomputed SVD-based pseudoinverse.
- In the coordinate frame fixed to wheel `i`, `W_i`, the axis `X_Wi` is aligned with the wheel rolling direction (motion can be along `+X_Wi` or `-X_Wi`).
- In the coordinate frame fixed to wheel `i`, `W_i`, the axis `Y_Wi` indicates the direction of the wheel rotation axis (rotation can be about `+Y_Wi` or `-Y_Wi`).
- Solver equations use only planar `XY` geometry (ground-plane model).

## License

This package is distributed under the Apache License 2.0.
See [LICENSE](LICENSE) for the complete terms.
