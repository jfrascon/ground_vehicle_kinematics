#pragma once

#include <limits>
#include <string>

namespace ground_vehicle_kinematics
{

  // Pose stores a 3D position and orientation using roll-pitch-yaw convention.
  struct Pose
  {
    double x{0.0};  // Position along the x-axis (meters).
    double y{0.0};  // Position along the y-axis (meters).
    double z{0.0};  // Position along the z-axis (meters).
    double R{0.0};  // Rotation about the x-axis (radians) - Roll.
    double P{0.0};  // Rotation about the y-axis (radians) - Pitch.
    double Y{0.0};  // Rotation about the z-axis (radians) - Yaw.
  };

  // Twist captures a spatial velocity; for planar kinematics we mainly rely on vx, vy, wz.
  struct Twist
  {
    double vx{std::numeric_limits<double>::quiet_NaN()};  // Linear velocity along x (m/s).
    double vy{std::numeric_limits<double>::quiet_NaN()};  // Linear velocity along y (m/s).
    double vz{std::numeric_limits<double>::quiet_NaN()};  // Linear velocity along z (m/s).
    double wx{std::numeric_limits<double>::quiet_NaN()};  // Angular velocity about x (rad/s).
    double wy{std::numeric_limits<double>::quiet_NaN()};  // Angular velocity about y (rad/s).
    double wz{std::numeric_limits<double>::quiet_NaN()};  // Angular velocity about z (rad/s).
  };

  // Limits represents a generic lower/upper bound pair and is reused for angular ranges.
  struct Limits
  {
    double lower{-3.14159265358979323846};  // Lower bound (radians for angular limits).
    double upper{3.14159265358979323846};   // Upper bound (radians for angular limits).
  };

  // RotationJoint describes the fixed steering axis for a driven wheel.
  struct RotationJoint
  {
    std::string name;              // Joint identifier used throughout the codebase.
    std::string parent_link_name;  // Name of the parent link in the robot model.
    std::string child_link_name;   // Name of the child link in the robot model.
    Pose origin;                   // Pose of the joint relative to the steering frame.
  };

  // SteerableJoint models the steering hinge that yaws the wheel module.
  struct SteerableJoint
  {
    std::string name;              // Joint identifier used in JointState messages.
    std::string parent_link_name;  // Parent link of the steering joint.
    std::string child_link_name;   // Child link of the steering joint.
    Pose origin;                   // Pose of the steering joint relative to the base.
    Limits angular_limits{};       // Valid steering range expressed in radians.
  };

  // Wheel stores the physical wheel data and its rotation joint.
  struct Wheel
  {
    std::string name;              // Human-readable name for diagnostics.
    double radius{0.0};            // Wheel radius (meters) used for velocity conversions.
    RotationJoint rotation_joint;  // Rotational joint responsible for tangential motion.
  };

  // SteerableWheel bundles a steerable joint with its driven wheel.
  struct SteerableWheel: public Wheel
  {
    SteerableJoint steerable_joint;
  };

  // RotationJointState represents the dynamic state of the wheel rotation joint.
  struct RotationJointState
  {
    std::string joint_name;  // Name used to match incoming JointState data.
    double theta{0.0};       // Joint position (radians).
    double alpha{0.0};       // Joint velocity (rad/s).
  };

  // WheelState stores the wheel rotation joint state for convenience.
  struct WheelState
  {
    std::string wheel_name;  // Name of the wheel for correlation.
    RotationJointState rotation_joint_state;
  };

  // SteerableJointState captures the steering joint state and saturation diagnostics.
  struct SteerableJointState
  {
    std::string joint_name;          // Name of the steering joint.
    double theta{0.0};               // Steering angle (radians).
    double alpha{0.0};               // Steering angular velocity (rad/s).
    bool steering_saturated{false};  // True if the solver clamped phi to stay within limits.
  };

  // SteerableWheelState combines steering and rotation states for a full module snapshot.
  struct SteerableWheelState: public WheelState
  {
    SteerableJointState steerable_joint_state;
  };

}  // namespace ground_vehicle_kinematics
