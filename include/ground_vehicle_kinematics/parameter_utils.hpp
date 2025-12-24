#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include "ground_vehicle_kinematics/core/types.hpp"

namespace ground_vehicle_kinematics
{
  Joint process_joint(const rclcpp::Node& node,
                      const std::string& robot_prefix,
                      const std::string& joint_parameter_path);

  Limits process_limits(const rclcpp::Node& node, const std::string& limits_path);

  Pose process_pose(const rclcpp::Node& node, const std::string& pose_path);

  Wheel process_wheel(const rclcpp::Node& node, const std::string& robot_prefix, const std::string& wheel_path);

  SteerableWheel process_st_wheel(const rclcpp::Node& node,
                                  const std::string& robot_prefix,
                                  const std::string& st_wheel_path);

  WheelDescriptor process_wheel_descriptor(const rclcpp::Node& node,
                                           const std::string& robot_prefix,
                                           const std::string& wheel_path);

  SteerableWheelDescriptor process_st_wheel_descriptor(const rclcpp::Node& node,
                                                       const std::string& robot_prefix,
                                                       const std::string& st_wheel_path);
}  // namespace ground_vehicle_kinematics
