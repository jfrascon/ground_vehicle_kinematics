#include "ground_vehicle_kinematics/parameter_utils.hpp"
#include <cmath>
#include <stdexcept>

namespace ground_vehicle_kinematics
{
  Joint process_joint(const rclcpp::Node& node, const std::string& joint_parameter_path)
  {
    if(joint_parameter_path.empty())
    {
      throw std::invalid_argument("Rotation joint path cannot be empty.");
    }

    const auto name{node.get_parameter(joint_parameter_path + ".name").as_string()};

    if(name.empty())
    {
      throw std::invalid_argument("Rotation joint name in path " + joint_parameter_path + " cannot be empty.");
    }

    const auto parent_link_name{node.get_parameter(joint_parameter_path + ".parent_link_name").as_string()};

    if(parent_link_name.empty())
    {
      throw std::invalid_argument("Rotation joint parent_link_name in path " + joint_parameter_path +
                                  " cannot be empty.");
    }

    const auto child_link_name{node.get_parameter(joint_parameter_path + ".child_link_name").as_string()};

    if(child_link_name.empty())
    {
      throw std::invalid_argument("Rotation joint child_link_name in path " + joint_parameter_path +
                                  " cannot be empty.");
    }

    return Joint{
      name,
      parent_link_name,
      child_link_name,
      process_pose(node, joint_parameter_path + ".origin"),
      process_limits(node, joint_parameter_path + ".limits"),
    };
  }

  //////////////////////////////////////////////////////////////////////////////

  Limits process_limits(const rclcpp::Node& node, const std::string& limits_path)
  {
    if(limits_path.empty())
    {
      throw std::invalid_argument("Limits path cannot be empty.");
    }

    return Limits{node.get_parameter(limits_path + ".lower").get_value<double>(),
                  node.get_parameter(limits_path + ".upper").get_value<double>()};
  }

  //////////////////////////////////////////////////////////////////////////////

  Pose process_pose(const rclcpp::Node& node, const std::string& pose_path)
  {
    if(pose_path.empty())
    {
      throw std::invalid_argument("Pose path cannot be empty.");
    }

    return Pose{
      node.get_parameter(pose_path + ".x").get_value<double>(),
      node.get_parameter(pose_path + ".y").get_value<double>(),
      node.get_parameter(pose_path + ".z").get_value<double>(),
      node.get_parameter(pose_path + ".R").get_value<double>(),
      node.get_parameter(pose_path + ".P").get_value<double>(),
      node.get_parameter(pose_path + ".Y").get_value<double>(),
    };
  }
  //////////////////////////////////////////////////////////////////////////////

  Wheel process_wheel(const rclcpp::Node& node, const std::string& wheel_path)
  {
    if(wheel_path.empty())
    {
      throw std::invalid_argument("Wheel path cannot be empty.");
    }

    const auto wheel_name{node.get_parameter(wheel_path + ".name").as_string()};

    if(wheel_name.empty())
    {
      throw std::invalid_argument("Wheel name in path " + wheel_path + " cannot be empty.");
    }

    const auto wheel_radius{node.get_parameter(wheel_path + ".radius").get_value<double>()};

    if(wheel_radius <= 0.0)
    {
      throw std::invalid_argument("Wheel radius in path " + wheel_path + " must be positive.");
    }

    return Wheel{wheel_name, wheel_radius, RotationJoint(process_joint(node, wheel_path + ".rotation_joint"))};
  }
  //////////////////////////////////////////////////////////////////////////////

  SteerableWheel process_st_wheel(const rclcpp::Node& node, const std::string& st_wheel_path)
  {
    SteerableWheel st_wheel{process_wheel(node, st_wheel_path),
                            SteerableJoint(process_joint(node, st_wheel_path + ".steerable_joint"))};

    if(st_wheel.rotation_joint.parent_link_name != st_wheel.steerable_joint.child_link_name)
    {
      throw std::invalid_argument(
        "In " + st_wheel_path + ": steerable_joint.child_link_name ('" + st_wheel.steerable_joint.child_link_name +
        "') must equal rotation_joint.parent_link_name ('" + st_wheel.rotation_joint.parent_link_name + "').");
    }

    const auto epsilon{1e-6};

    if(std::abs(st_wheel.rotation_joint.origin.x) > epsilon || std::abs(st_wheel.rotation_joint.origin.y) > epsilon ||
       std::abs(st_wheel.rotation_joint.origin.R) > epsilon || std::abs(st_wheel.rotation_joint.origin.P) > epsilon ||
       std::abs(st_wheel.rotation_joint.origin.Y) > epsilon)
    {
      throw std::invalid_argument(
        "In " + st_wheel_path +
        ": rotation_joint pose must have x=0, y=0, R=0, P=0, Y=0. Only z translation is allowed. Got: x=" +
        std::to_string(st_wheel.rotation_joint.origin.x) + ", y=" + std::to_string(st_wheel.rotation_joint.origin.y) +
        ", R=" + std::to_string(st_wheel.rotation_joint.origin.R) + ", P=" +
        std::to_string(st_wheel.rotation_joint.origin.P) + ", Y=" + std::to_string(st_wheel.rotation_joint.origin.Y));
    }

    return st_wheel;
  }
  //////////////////////////////////////////////////////////////////////////////

  WheelDescriptor process_wheel_descriptor(const rclcpp::Node& node, const std::string& wheel_path)
  {
    WheelDescriptor wheel_desc;
    wheel_desc.wheel                                           = process_wheel(node, wheel_path);
    wheel_desc.wheel_state.wheel_name                          = wheel_desc.wheel.name;
    wheel_desc.wheel_state.rotation_joint_state.joint_name     = wheel_desc.wheel.rotation_joint.name;
    wheel_desc.wheel_command.wheel_name                        = wheel_desc.wheel.name;
    wheel_desc.wheel_command.rotation_joint_command.joint_name = wheel_desc.wheel.rotation_joint.name;

    return wheel_desc;
  }
  //////////////////////////////////////////////////////////////////////////////

  SteerableWheelDescriptor process_st_wheel_descriptor(const rclcpp::Node& node, const std::string& st_wheel_path)
  {
    SteerableWheelDescriptor st_wheel_desc;

    st_wheel_desc.st_wheel                                            = process_st_wheel(node, st_wheel_path);
    st_wheel_desc.st_wheel_state.wheel_name                           = st_wheel_desc.st_wheel.name;
    st_wheel_desc.st_wheel_state.rotation_joint_state.joint_name      = st_wheel_desc.st_wheel.rotation_joint.name;
    st_wheel_desc.st_wheel_state.steerable_joint_state.joint_name     = st_wheel_desc.st_wheel.steerable_joint.name;
    st_wheel_desc.st_wheel_command.wheel_name                         = st_wheel_desc.st_wheel.name;
    st_wheel_desc.st_wheel_command.rotation_joint_command.joint_name  = st_wheel_desc.st_wheel.rotation_joint.name;
    st_wheel_desc.st_wheel_command.steerable_joint_command.joint_name = st_wheel_desc.st_wheel.steerable_joint.name;

    return st_wheel_desc;
  }
}  // namespace ground_vehicle_kinematics
