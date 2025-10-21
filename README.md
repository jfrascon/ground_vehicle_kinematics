# ground_vehicle_kinematics

This package provides utilities and nodes to compute forward and inverse kinematics for ground vehicles. The current focus is on swerve-drive (steerable wheel) platforms, but the architecture is designed to support additional vehicle models in the future.

## Main Features

- Implements forward and inverse kinematics algorithms for ground vehicles.
- Includes ready-to-use ROS 2 nodes written in C++.
- Integrates with other ROS 2 packages using standard messages (`geometry_msgs`, `sensor_msgs`).
- Uses Eigen3 for efficient mathematical computations.
- Provides configuration files and launch scripts for easy setup and testing.
- Architecture prepared to support various types of ground vehicle kinematics.

## Package Structure

- `src/`: Main source code for nodes and utilities.
- `include/`: Public headers for use by other packages.
- `launch/`: Launch files to start nodes and configurations.
- `config/`: Example YAML configuration files.

## Typical Usage

This package serves as a foundation for controlling and simulating the kinematics of mobile robots, enabling the conversion of velocity commands into wheel movements and vice versa, and facilitating integration with navigation and control systems for ground vehicles.
