#pragma once

#include <limits>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace ground_vehicle_kinematics
{
  // Limits represents a generic lower/upper bound pair and is reused for angular ranges.
  struct Limits
  {
    double lower{-3.14159265358979323846};  // Lower bound (rad for angular limits, rad/s for velocity limits).
    double upper{3.14159265358979323846};   // Upper bound (rad for angular limits, rad/s for velocity limits).

    Limits() = default;
    Limits(double lower, double upper): lower{lower}, upper{upper} {}
    Limits(const Limits&) = default;
  };

  // Pose stores a 3D position and orientation using roll-pitch-yaw convention.
  struct Pose
  {
    double x{0.0};  // Position along the x-axis (meters).
    double y{0.0};  // Position along the y-axis (meters).
    double z{0.0};  // Position along the z-axis (meters).
    double R{0.0};  // Rotation about the x-axis (radians) - Roll.
    double P{0.0};  // Rotation about the y-axis (radians) - Pitch.
    double Y{0.0};  // Rotation about the z-axis (radians) - Yaw.

    Pose() = default;
    Pose(double x, double y, double z, double R, double P, double Y): x{x}, y{y}, z{z}, R{R}, P{P}, Y{Y} {}
    Pose(const Pose&) = default;
  };

  // Twist captures a spatial velocity; for planar kinematics we mainly rely on vx, vy, wz.
  struct Twist
  {
    double vx{0.0};  // Linear velocity along x (m/s).
    double vy{0.0};  // Linear velocity along y (m/s).
    double vz{0.0};  // Linear velocity along z (m/s).
    double wx{0.0};  // Angular velocity about x (rad/s).
    double wy{0.0};  // Angular velocity about y (rad/s).
    double wz{0.0};  // Angular velocity about z (rad/s).

    Twist() = default;
    Twist(double vx, double vy, double vz, double wx, double wy, double wz):
      vx{vx},
      vy{vy},
      vz{vz},
      wx{wx},
      wy{wy},
      wz{wz}
    {}
    Twist(const Twist&) = default;
  };

  //////////////////////////////////////////////////////////////////////////////
  // JOINTS
  //////////////////////////////////////////////////////////////////////////////

  // RotationJoint describes the fixed steering axis for a driven wheel.
  struct Joint
  {
    std::string name;              // Joint identifier used throughout the codebase.
    std::string parent_link_name;  // Name of the parent link in the robot model.
    std::string child_link_name;   // Name of the child link in the robot model.
    Pose origin;                   // Pose of the joint relative to the steering frame.
    Limits limits;                 // Joint limits (rad/s for rotation joints and rads for steerable joints).

    Joint() = default;

    Joint(const std::string& name,
          const std::string& parent_link_name,
          const std::string& child_link_name,
          const Pose& origin,
          const Limits& limits):
      name(name),
      parent_link_name(parent_link_name),
      child_link_name(child_link_name),
      origin(origin),
      limits(limits)
    {}

    Joint(const Joint&) = default;
  };

  //////////////////////////////////////////////////////////////////////////////

  struct RotationJoint: public Joint
  {
    RotationJoint() = default;

    RotationJoint(const std::string& name,
                  const std::string& parent_link_name,
                  const std::string& child_link_name,
                  const Pose& origin,
                  const Limits& limits):
      Joint(name, parent_link_name, child_link_name, origin, limits)
    {}

    RotationJoint(const Joint& joint): Joint(joint) {}

    RotationJoint(const RotationJoint&) = default;
  };

  //////////////////////////////////////////////////////////////////////////////

  // SteerableJoint models the steering hinge that yaws the wheel module.
  struct SteerableJoint: public Joint
  {
    SteerableJoint() = default;

    SteerableJoint(const std::string& name,
                   const std::string& parent_link_name,
                   const std::string& child_link_name,
                   const Pose& origin,
                   const Limits& limits):
      Joint(name, parent_link_name, child_link_name, origin, limits)
    {}

    SteerableJoint(const Joint& joint): Joint(joint) {}

    SteerableJoint(const SteerableJoint&) = default;
  };

  //////////////////////////////////////////////////////////////////////////////
  // WHEELS
  //////////////////////////////////////////////////////////////////////////////

  // Wheel stores the physical wheel data and its rotation joint.
  struct Wheel
  {
    std::string name;              // Human-readable name for diagnostics.
    double radius{0.0};            // Wheel radius (meters) used for velocity conversions.
    RotationJoint rotation_joint;  // Rotational joint responsible for tangential motion.

    Wheel() = default;

    Wheel(const std::string& name, double radius, const RotationJoint& rj):
      name(name),
      radius(radius),
      rotation_joint(rj)
    {}

    Wheel(const Wheel&) = default;
  };

  //////////////////////////////////////////////////////////////////////////////

  struct SteerableWheel: public Wheel
  {
    SteerableJoint steerable_joint;

    SteerableWheel() = default;

    SteerableWheel(const std::string& name, double radius, const RotationJoint& rj, const SteerableJoint& sj):
      Wheel(name, radius, rj),
      steerable_joint(sj)
    {}

    SteerableWheel(const Wheel& w, const SteerableJoint& sj): Wheel(w), steerable_joint(sj) {}

    SteerableWheel(const SteerableWheel&) = default;
  };

  //////////////////////////////////////////////////////////////////////////////
  // States
  //////////////////////////////////////////////////////////////////////////////

  struct JointState
  {
    std::string joint_name;
    double angle{0.0};    // Joint position (radians).
    double ang_vel{0.0};  // Joint velocity (rad/s).

    JointState() = default;

    JointState(const std::string& joint_name, double angle = 0.0, double ang_vel = 0.0):
      joint_name(joint_name),
      angle(angle),
      ang_vel(ang_vel)
    {}

    JointState(const JointState&) = default;
  };

  //////////////////////////////////////////////////////////////////////////////

  // RotationJointState does not add any new fields for now.
  struct RotationJointState: public JointState
  {
    RotationJointState() = default;

    RotationJointState(const std::string& joint_name, double angle = 0.0, double ang_vel = 0.0):
      JointState(joint_name, angle, ang_vel)
    {}

    RotationJointState(const JointState& js): JointState(js) {}

    RotationJointState(const RotationJointState&) = default;
  };

  //////////////////////////////////////////////////////////////////////////////

  // SteerableJointState does not add any new fields for now.
  struct SteerableJointState: public JointState
  {
    SteerableJointState() = default;

    SteerableJointState(const std::string& joint_name, double angle = 0.0, double ang_vel = 0.0):
      JointState(joint_name, angle, ang_vel)
    {}

    SteerableJointState(const JointState& js): JointState(js) {}

    SteerableJointState(const SteerableJointState&) = default;
  };

  //////////////////////////////////////////////////////////////////////////////

  struct WheelState
  {
    std::string wheel_name;  // Name of the wheel for correlation.
    RotationJointState rotation_joint_state;

    WheelState() = default;

    WheelState(const std::string& wheel_name, const std::string& joint_name):
      wheel_name(wheel_name),
      rotation_joint_state(joint_name)
    {}

    WheelState(const std::string& wheel_name, const RotationJointState& rjs):
      wheel_name(wheel_name),
      rotation_joint_state(rjs)
    {}

    WheelState(const WheelState&) = default;
  };

  //////////////////////////////////////////////////////////////////////////////

  struct SteerableWheelState: public WheelState
  {
    SteerableJointState steerable_joint_state;

    SteerableWheelState() = default;

    SteerableWheelState(const std::string& wheel_name,
                        const std::string& rotation_joint_name,
                        const std::string& steerable_joint_name):
      WheelState(wheel_name, rotation_joint_name),
      steerable_joint_state(steerable_joint_name)
    {}

    SteerableWheelState(const std::string& wheel_name, const RotationJointState& rjs, const SteerableJointState& sjs):
      WheelState(wheel_name, rjs),
      steerable_joint_state(sjs)
    {}

    SteerableWheelState(const WheelState& ws, const SteerableJointState& sjs):
      WheelState(ws),
      steerable_joint_state(sjs)
    {}
  };

  //////////////////////////////////////////////////////////////////////////////
  // Commands
  //////////////////////////////////////////////////////////////////////////////

  struct JointCommand
  {
    std::string joint_name;
    double value{0.0};  // In rad for steerable joints, rad/s for rotation joints.

    JointCommand() = default;

    JointCommand(const std::string& joint_name, double value = 0): joint_name(joint_name), value(value) {}

    JointCommand(const JointCommand&) = default;
  };

  //////////////////////////////////////////////////////////////////////////////

  // RotationJointCommand does not add any new fields for now.
  struct RotationJointCommand: public JointCommand
  {
    RotationJointCommand() = default;

    RotationJointCommand(const std::string& joint_name, double value = 0): JointCommand(joint_name, value) {}

    RotationJointCommand(const JointCommand& jc): JointCommand(jc) {}

    RotationJointCommand(const RotationJointCommand&) = default;
  };

  //////////////////////////////////////////////////////////////////////////////

  // SteerableJointCommand does not add any new fields for now.
  struct SteerableJointCommand: public JointCommand
  {
    SteerableJointCommand() = default;

    SteerableJointCommand(const std::string& joint_name, double value = 0): JointCommand(joint_name, value) {}

    SteerableJointCommand(const JointCommand& jc): JointCommand(jc) {}

    SteerableJointCommand(const SteerableJointCommand&) = default;
  };

  //////////////////////////////////////////////////////////////////////////////

  struct WheelCommand
  {
    std::string wheel_name;
    RotationJointCommand rotation_joint_command;

    WheelCommand() = default;

    WheelCommand(const std::string& wheel_name, const RotationJointCommand& rjc):
      wheel_name(wheel_name),
      rotation_joint_command(rjc)
    {}

    WheelCommand(const WheelCommand&) = default;
  };

  //////////////////////////////////////////////////////////////////////////////

  struct SteerableWheelCommand: public WheelCommand
  {
    SteerableJointCommand steerable_joint_command;

    SteerableWheelCommand() = default;

    SteerableWheelCommand(const std::string& wheel_name,
                          const RotationJointCommand& rjc,
                          const SteerableJointCommand& sjc):
      WheelCommand(wheel_name, rjc),
      steerable_joint_command(sjc)
    {}

    SteerableWheelCommand(const WheelCommand& wc, const SteerableJointCommand& sjc):
      WheelCommand(wc),
      steerable_joint_command(sjc)
    {}
  };

  //////////////////////////////////////////////////////////////////////////////

  struct WheelDescriptor
  {
    Wheel wheel;
    WheelState wheel_state;
    uint64_t t_wheel_state{0};
    WheelCommand wheel_command;
    uint64_t t_wheel_command{0};

    WheelDescriptor() = default;

    WheelDescriptor(const Wheel& wheel,
                    const WheelState& wheel_state,
                    uint64_t t_wheel_state,
                    const WheelCommand& wheel_command,
                    uint64_t t_wheel_command):
      wheel(wheel),
      wheel_state(wheel_state),
      t_wheel_state(t_wheel_state),
      wheel_command(wheel_command),
      t_wheel_command(t_wheel_command)
    {}

    WheelDescriptor(const WheelDescriptor&) = default;
  };

  struct SteerableWheelDescriptor
  {
    SteerableWheel st_wheel;
    SteerableWheelState st_wheel_state;
    uint64_t t_st_wheel_state{0};
    SteerableWheelCommand st_wheel_command;
    uint64_t t_st_wheel_command{0};

    SteerableWheelDescriptor() = default;

    SteerableWheelDescriptor(const SteerableWheel& st_wheel,
                             const SteerableWheelState& st_wheel_state,
                             uint64_t t_st_wheel_state,
                             const SteerableWheelCommand& st_wheel_command,
                             uint64_t t_st_wheel_command):
      st_wheel(st_wheel),
      st_wheel_state(st_wheel_state),
      t_st_wheel_state(t_st_wheel_state),
      st_wheel_command(st_wheel_command),
      t_st_wheel_command(t_st_wheel_command)
    {}

    SteerableWheelDescriptor(const SteerableWheelDescriptor&) = default;
  };
}  // namespace ground_vehicle_kinematics
