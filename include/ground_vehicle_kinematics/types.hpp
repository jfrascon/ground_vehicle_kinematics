#pragma once

/**
 * @file types.hpp
 * @brief Strongly-typed data model for three-swerve geometry, states, commands and descriptors.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "ground_vehicle_kinematics/helpers.hpp"

namespace ground_vehicle_kinematics
{
  /**
   * @brief Lower and upper limits used by steering-angle and wheel-speed constraints.
   */
  class Limits
  {
    public:
    /**
     * @brief Build limits and validate interval order.
     * @param lower Lower boundary.
     * @param upper Upper boundary.
     * @throws std::invalid_argument If @p lower is greater than @p upper.
     */
    Limits(const double lower = 0.0, const double upper = 0.0): lower_{lower}, upper_{upper}
    {
      validate();
    }

    /**
     * @brief Check whether a scalar value lies within these limits using a small tolerance.
     * @param value Scalar value to test.
     * @param tolerance Numerical tolerance used to absorb floating-point roundoff at the boundaries.
     * @return True if @p value lies within [lower, upper], accounting for @p tolerance.
     */
    bool contains(const double value, const double tolerance = 1e-9) const
    {
      if(!std::isfinite(value) || !std::isfinite(tolerance))
      {
        return false;
      }

      // Use a small tolerance around the limits. This avoids rejecting values that only differ from the exact
      // boundary because of floating-point roundoff.
      return (value > (lower_ - tolerance)) && (value < (upper_ + tolerance));
    }

    /** @return Lower boundary. */
    double lower() const
    {
      return lower_;
    }

    /** @return Upper boundary. */
    double upper() const
    {
      return upper_;
    }

    private:
    double lower_;  ///< Lower limit bound.
    double upper_;  ///< Upper limit bound.

    /**
     * @brief Validate interval order invariant.
     * @throws std::invalid_argument If lower bound is greater than upper bound.
     */
    void validate() const
    {
      detail::ensure_finite(lower_, "Limits.lower");
      detail::ensure_finite(upper_, "Limits.upper");

      if(lower_ > upper_)
      {
        throw std::invalid_argument("Invalid limits: lower cannot be greater than upper.");
      }
    }
  };

  /**
   * @brief Planar position used by the kinematic equations.
   */
  class Position2D
  {
    public:
    /**
     * @brief Build a 2D position.
     * @param x Position in X (m).
     * @param y Position in Y (m).
     * @throws std::invalid_argument If any coordinate is not finite.
     */
    Position2D(const double x = 0.0, const double y = 0.0): x_{x}, y_{y}
    {
      validate();
    }

    /** @return X coordinate (m). */
    double x() const
    {
      return x_;
    }

    /** @return Y coordinate (m). */
    double y() const
    {
      return y_;
    }

    private:
    double x_;  ///< Position in X (m).
    double y_;  ///< Position in Y (m).

    /**
     * @brief Validate coordinate values.
     * @throws std::invalid_argument If any coordinate is not finite.
     */
    void validate() const
    {
      detail::ensure_finite(x_, "Position2D.x");
      detail::ensure_finite(y_, "Position2D.y");
    }
  };

  /**
   * @brief Spatial twist representation.
   *
   * For ground vehicles, the relevant components are typically vx, vy and wz.
   */
  class Twist
  {
    public:
    /**
     * @brief Build full 6D twist.
     * @param vx Linear velocity in X (m/s).
     * @param vy Linear velocity in Y (m/s).
     * @param vz Linear velocity in Z (m/s).
     * @param wx Angular velocity around X (rad/s).
     * @param wy Angular velocity around Y (rad/s).
     * @param wz Angular velocity around Z (rad/s).
     * @param timestamp_ns Timestamp associated with this twist.
     */
    Twist(const double vx            = 0.0,
          const double vy            = 0.0,
          const double vz            = 0.0,
          const double wx            = 0.0,
          const double wy            = 0.0,
          const double wz            = 0.0,
          const int64_t timestamp_ns = 0):
      vx_{vx},
      vy_{vy},
      vz_{vz},
      wx_{wx},
      wy_{wy},
      wz_{wz},
      timestamp_ns_{timestamp_ns}
    {
      validate();
    }

    /** @return Linear velocity in X (m/s). */
    double vx() const
    {
      return vx_;
    }

    /** @return Linear velocity in Y (m/s). */
    double vy() const
    {
      return vy_;
    }

    /** @return Linear velocity in Z (m/s). */
    double vz() const
    {
      return vz_;
    }

    /** @return Angular velocity around X (rad/s). */
    double wx() const
    {
      return wx_;
    }

    /** @return Angular velocity around Y (rad/s). */
    double wy() const
    {
      return wy_;
    }

    /** @return Angular velocity around Z (rad/s). */
    double wz() const
    {
      return wz_;
    }

    /** @return Timestamp associated with this twist. */
    int64_t timestamp_ns() const
    {
      return timestamp_ns_;
    }

    private:
    double vx_;             ///< Linear velocity in X (m/s).
    double vy_;             ///< Linear velocity in Y (m/s).
    double vz_;             ///< Linear velocity in Z (m/s).
    double wx_;             ///< Angular velocity around X (rad/s).
    double wy_;             ///< Angular velocity around Y (rad/s).
    double wz_;             ///< Angular velocity around Z (rad/s).
    int64_t timestamp_ns_;  ///< Timestamp associated with this twist.

    /**
     * @brief Validate twist components.
     * @throws std::invalid_argument If any component is not finite.
     */
    void validate() const
    {
      detail::ensure_finite(vx_, "Twist.vx");
      detail::ensure_finite(vy_, "Twist.vy");
      detail::ensure_finite(vz_, "Twist.vz");
      detail::ensure_finite(wx_, "Twist.wx");
      detail::ensure_finite(wy_, "Twist.wy");
      detail::ensure_finite(wz_, "Twist.wz");
    }
  };

  //////////////////////////////////////////////////////////////////////////////
  // CONFIGS
  //////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Wheel configuration used to build one wheel model.
   */
  class WheelConfig
  {
    public:
    /**
     * @brief Build one wheel config.
     * @param radius Wheel radius (m).
     * @param dist Distance from the base origin to the wheel position.
     * @param alpha Wheel position angle in radians.
     * @param beta Wheel base orientation angle in radians.
     * @param wheel_name Wheel name.
     * @param rotation_joint_name Rotation-joint name.
     * @param rotation_joint_lower Rotation-joint lower limit.
     * @param rotation_joint_upper Rotation-joint upper limit.
     */
    WheelConfig(const double radius,
                const double dist,
                const double alpha,
                const double beta,
                const std::string& wheel_name,
                const std::string& rotation_joint_name,
                const double rotation_joint_lower,
                const double rotation_joint_upper):
      radius_{radius},
      dist_{dist},
      alpha_{alpha},
      beta_{beta},
      wheel_name_{wheel_name},
      rotation_joint_name_{rotation_joint_name},
      rotation_joint_lower_{rotation_joint_lower},
      rotation_joint_upper_{rotation_joint_upper}
    {}

    /** @return Wheel radius (m). */
    double alpha() const
    {
      return alpha_;
    }

    /** @return Wheel base orientation angle (rad). */
    double beta() const
    {
      return beta_;
    }

    /** @return Distance from the base origin to the wheel position. */
    double dist() const
    {
      return dist_;
    }

    /** @return Wheel radius (m). */
    double radius() const
    {
      return radius_;
    }

    /** @return Rotation-joint lower limit. */
    double rotation_joint_lower() const
    {
      return rotation_joint_lower_;
    }

    /** @return Rotation-joint name. */
    const std::string& rotation_joint_name() const
    {
      return rotation_joint_name_;
    }

    /** @return Rotation-joint upper limit. */
    double rotation_joint_upper() const
    {
      return rotation_joint_upper_;
    }

    /** @return Wheel name. */
    const std::string& wheel_name() const
    {
      return wheel_name_;
    }

    protected:
    double radius_;
    double dist_;
    double alpha_;
    double beta_;
    std::string wheel_name_;
    std::string rotation_joint_name_;
    double rotation_joint_lower_;
    double rotation_joint_upper_;
  };

  /**
   * @brief Steerable-wheel configuration used to build one steerable wheel model.
   */
  class SteerableWheelConfig: public WheelConfig
  {
    public:
    /**
     * @brief Build one steerable-wheel config.
     * @param radius Wheel radius (m).
     * @param dist Distance from the base origin to the wheel position.
     * @param alpha Wheel position angle in radians.
     * @param beta Wheel base orientation angle in radians.
     * @param wheel_name Wheel name.
     * @param rotation_joint_name Rotation-joint name.
     * @param rotation_joint_lower Rotation-joint lower limit.
     * @param rotation_joint_upper Rotation-joint upper limit.
     * @param steering_joint_name Steering-joint name.
     * @param steering_joint_lower Steering-joint lower limit.
     * @param steering_joint_upper Steering-joint upper limit.
     */
    SteerableWheelConfig(const double radius,
                         const double dist,
                         const double alpha,
                         const double beta,
                         const std::string& wheel_name,
                         const std::string& rotation_joint_name,
                         const double rotation_joint_lower,
                         const double rotation_joint_upper,
                         const std::string& steering_joint_name,
                         const double steering_joint_lower,
                         const double steering_joint_upper):
      WheelConfig(radius,
                  dist,
                  alpha,
                  beta,
                  wheel_name,
                  rotation_joint_name,
                  rotation_joint_lower,
                  rotation_joint_upper),
      steering_joint_name_{steering_joint_name},
      steering_joint_lower_{steering_joint_lower},
      steering_joint_upper_{steering_joint_upper}
    {}

    /** @return Steering-joint lower limit. */
    double steering_joint_lower() const
    {
      return steering_joint_lower_;
    }

    /** @return Steering-joint name. */
    const std::string& steering_joint_name() const
    {
      return steering_joint_name_;
    }

    /** @return Steering-joint upper limit. */
    double steering_joint_upper() const
    {
      return steering_joint_upper_;
    }

    private:
    std::string steering_joint_name_;
    double steering_joint_lower_{0.0};
    double steering_joint_upper_{0.0};
  };

  //////////////////////////////////////////////////////////////////////////////
  // JOINTS
  //////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Joint model used for steer and rotation joints.
   */
  class Joint
  {
    public:
    /**
     * @brief Build a joint model.
     * @param name Joint name.
     * @param limits Joint limits.
     * @throws std::invalid_argument If @p name is empty.
     */
    Joint(const std::string& name, const Limits& limits): name_{name}, limits_{limits}
    {
      validate();
    }

    /** @return Joint name. */
    const std::string& name() const
    {
      return name_;
    }

    /** @return Joint limits. */
    const Limits& limits() const
    {
      return limits_;
    }

    protected:
    std::string name_;  ///< Joint name.
    Limits limits_;     ///< Joint limits.

    private:
    void validate() const
    {
      detail::ensure_non_empty(name_, "Joint.name");
    }
  };

  /** @brief Rotation joint model. */
  class RotationJoint: public Joint
  {
    public:
    /**
     * @brief Build a rotation-joint descriptor.
     * @param name Rotation-joint name.
     * @param limits Rotation-joint limits.
     */
    RotationJoint(const std::string& name, const Limits& limits): Joint(name, limits) {}
  };

  /** @brief Steering joint model. */
  class SteeringJoint: public Joint
  {
    public:
    /**
     * @brief Build a steering-joint descriptor.
     * @param name Steering-joint name.
     * @param limits Steering limits.
     * @throws std::invalid_argument If @p limits lie outside [-2pi, 2pi].
     */
    SteeringJoint(const std::string& name, const Limits& limits): Joint(name, limits)
    {
      validate();
    }

    private:
    /**
     * @brief Maximum absolute steering limit accepted by the data model.
     *
     * This package assumes that, for kinematic purposes, a steerable wheel should not need more than one full turn
     * to either side of its nominal orientation. That leads to the bound [-2pi, 2pi].
     *
     * If a real steering actuator can physically rotate more than one full turn in either direction, the
     * configuration passed to this package should still be limited to the equivalent interval within [-2pi, 2pi]
     * that is meaningful for the kinematic model.
     */
    static constexpr double max_abs_limit_{2.0 * M_PI};

    void validate() const
    {
      if(limits_.lower() < -max_abs_limit_ || limits_.upper() > max_abs_limit_)
      {
        throw std::invalid_argument("SteeringJoint.limits must lie within [-2pi, 2pi].");
      }
    }
  };

  //////////////////////////////////////////////////////////////////////////////
  // WHEELS
  //////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Wheel model with planar position, geometry scalars, radius and associated rotation joint.
   */
  class Wheel
  {
    public:
    /**
     * @brief Build a wheel model.
     * @param radius Wheel radius (m).
     * @param dist Distance from the base origin to the wheel position.
     * @param alpha Wheel position angle in radians.
     * @param beta Wheel base orientation angle in radians.
     * @param name Wheel name.
     * @param rotation_joint Rotation joint model.
     * @throws std::invalid_argument If any stored scalar is invalid.
     */
    Wheel(const double radius,
          const double dist,
          const double alpha,
          const double beta,
          const std::string& name,
          const RotationJoint& rotation_joint):
      name_{name},
      radius_{radius},
      dist_{dist},
      alpha_{alpha},
      beta_{beta},
      rotation_joint_{rotation_joint}
    {
      validate();
      pos2d_ = Position2D{dist_ * std::cos(alpha_), dist_ * std::sin(alpha_)};
    }

    /**
     * @brief Get wheel name.
     * @return Wheel name.
     */
    const std::string& name() const
    {
      return name_;
    }

    /**
     * @brief Get wheel radius.
     * @return Wheel radius (m).
     */
    double radius() const
    {
      return radius_;
    }

    /**
     * @brief Get distance from the base origin to the wheel position.
     * @return Distance from the base origin to the wheel position.
     */
    double dist() const
    {
      return dist_;
    }

    /**
     * @brief Get wheel position angle.
     * @return Wheel position angle in radians.
     */
    double alpha() const
    {
      return alpha_;
    }

    /**
     * @brief Get wheel base orientation angle.
     * @return Wheel base orientation angle in radians.
     */
    double beta() const
    {
      return beta_;
    }

    /**
     * @brief Get wheel planar position.
     * @return Wheel planar position.
     */
    const Position2D& pos2d() const
    {
      return pos2d_;
    }

    /**
     * @brief Get rotation-joint descriptor.
     * @return Const reference to rotation-joint descriptor.
     */
    const RotationJoint& rotation_joint() const
    {
      return rotation_joint_;
    }

    private:
    std::string name_;              ///< Wheel name.
    double radius_{0.0};            ///< Wheel radius (m).
    double dist_{0.0};              ///< Distance from the base origin to the wheel position.
    double alpha_{0.0};             ///< Wheel position angle (rad).
    double beta_{0.0};              ///< Wheel base orientation angle (rad).
    Position2D pos2d_;              ///< Wheel planar position used by the solver.
    RotationJoint rotation_joint_;  ///< Rotation joint descriptor.

    void validate() const
    {
      detail::ensure_non_empty(name_, "Wheel.name");
      detail::ensure_positive_finite(radius_, "Wheel.radius");
      detail::ensure_positive_finite(dist_, "Wheel.dist");
      detail::ensure_finite(alpha_, "Wheel.alpha");
      detail::ensure_finite(beta_, "Wheel.beta");
    }
  };

  /**
   * @brief Steerable wheel model: wheel + steering joint.
   */
  class SteerableWheel: public Wheel
  {
    public:
    /**
     * @brief Build steerable wheel model.
     */
    SteerableWheel(const double radius,
                   const double dist,
                   const double alpha,
                   const double beta,
                   const std::string& name,
                   const RotationJoint& rotation_joint,
                   const SteeringJoint& steering_joint):
      Wheel(radius, dist, alpha, beta, name, rotation_joint),
      steering_joint_{steering_joint}
    {}

    /**
     * @brief Get steering-joint descriptor.
     * @return Const reference to steering-joint descriptor.
     */
    const SteeringJoint& steering_joint() const
    {
      return steering_joint_;
    }

    private:
    SteeringJoint steering_joint_;  ///< Steering joint descriptor.
  };

  //////////////////////////////////////////////////////////////////////////////
  // STATES
  //////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Generic joint state (angle and angular velocity).
   */
  class JointState
  {
    public:
    /** @brief Build an empty joint state. */
    JointState() = default;

    /**
     * @brief Build a joint state.
     * @param joint_name Joint name.
     * @param angle Joint angle (rad).
     * @param ang_vel Joint angular velocity (rad/s).
     * @throws std::invalid_argument If @p joint_name is empty.
     */
    JointState(const std::string& joint_name,
               const double angle         = 0,
               const double ang_vel       = 0,
               const int64_t timestamp_ns = 0L):
      joint_name_{joint_name},
      angle_{angle},
      ang_vel_{ang_vel},
      timestamp_ns_{timestamp_ns}
    {
      validate();
    }

    /**
     * @brief Get joint name.
     * @return Joint name.
     */
    const std::string& joint_name() const
    {
      return joint_name_;
    }

    /**
     * @brief Get joint angle.
     * @return Joint angle (rad).
     */
    double angle() const
    {
      return angle_;
    }

    /**
     * @brief Get joint angular velocity.
     * @return Joint angular velocity (rad/s).
     */
    double ang_vel() const
    {
      return ang_vel_;
    }

    /**
     * @brief Get state timestamp.
     * @return Timestamp associated with this joint state.
     */
    int64_t timestamp_ns() const
    {
      return timestamp_ns_;
    }

    private:
    void validate() const
    {
      detail::ensure_non_empty(joint_name_, "JointState.joint_name");
      detail::ensure_finite(angle_, "JointState.angle");
      detail::ensure_finite(ang_vel_, "JointState.ang_vel");
    }

    std::string joint_name_;   ///< Joint name.
    double angle_{0.0};        ///< Joint angle (rad).
    double ang_vel_{0.0};      ///< Joint angular velocity (rad/s).
    int64_t timestamp_ns_{0};  ///< Timestamp associated with this joint state.
  };

  /** @brief Rotation-joint state specialization. */
  class RotationJointState: public JointState
  {
    public:
    /** @brief Build an empty rotation-joint state. */
    RotationJointState() = default;

    /**
     * @brief Build a rotation-joint state.
     * @param joint_name Rotation-joint name.
     * @param angle Joint angle (rad).
     * @param ang_vel Joint angular velocity (rad/s).
     */
    RotationJointState(const std::string& joint_name,
                       const double angle,
                       const double ang_vel,
                       const int64_t timestamp_ns):
      JointState(joint_name, angle, ang_vel, timestamp_ns)
    {}
  };

  /** @brief Steering-joint state specialization. */
  class SteeringJointState: public JointState
  {
    public:
    /** @brief Build an empty steering-joint state. */
    SteeringJointState() = default;

    /**
     * @brief Build a steering-joint state.
     * @param joint_name Steering-joint name.
     * @param angle Joint angle (rad).
     * @param ang_vel Joint angular velocity (rad/s).
     */
    SteeringJointState(const std::string& joint_name,
                       const double angle,
                       const double ang_vel,
                       const int64_t timestamp_ns):
      JointState(joint_name, angle, ang_vel, timestamp_ns)
    {}
  };

  /**
   * @brief Runtime state of a wheel module.
   */
  class WheelState
  {
    public:
    /** @brief Build an empty wheel state. */
    WheelState() = default;

    /**
     * @brief Build wheel state from rotation-joint state.
     * @param wheel_name Wheel name.
     * @param rotation_joint_state Rotation-joint state.
     * @throws std::invalid_argument If @p wheel_name is empty.
     */
    WheelState(const std::string& wheel_name, const RotationJointState& rotation_joint_state):
      wheel_name_{wheel_name},
      rotation_joint_state_{rotation_joint_state}
    {
      validate();
    }

    bool matches(const Wheel& wheel) const
    {
      return wheel_name_ == wheel.name() && rotation_joint_state_.joint_name() == wheel.rotation_joint().name() &&
             wheel.rotation_joint().limits().contains(rotation_joint_state_.ang_vel());
    }

    /**
     * @brief Get rotation-joint state.
     * @return Const reference to rotation-joint state.
     */
    const RotationJointState& rotation_joint_state() const
    {
      return rotation_joint_state_;
    }

    /**
     * @brief Get wheel name.
     * @return Wheel name.
     */
    const std::string& wheel_name() const
    {
      return wheel_name_;
    }

    private:
    void validate() const
    {
      detail::ensure_non_empty(wheel_name_, "WheelState.wheel_name");
    }

    std::string wheel_name_;                   ///< Wheel name.
    RotationJointState rotation_joint_state_;  ///< Rotation-joint state.
  };

  /**
   * @brief Runtime state of a steerable wheel module.
   */
  class SteerableWheelState: public WheelState
  {
    public:
    /** @brief Build an empty steerable-wheel state. */
    SteerableWheelState() = default;

    /**
     * @brief Build steerable-wheel state from wheel and steer states.
     * @param wheel_name Wheel name.
     * @param rotation_joint_state Rotation-joint state.
     * @param steering_joint_state Steering-joint state.
     */
    SteerableWheelState(const std::string& wheel_name,
                        const RotationJointState& rotation_joint_state,
                        const SteeringJointState& steering_joint_state):
      WheelState(wheel_name, rotation_joint_state),
      steering_joint_state_{steering_joint_state}
    {}

    bool matches(const SteerableWheel& wheel) const
    {
      return WheelState::matches(wheel) && steering_joint_state_.joint_name() == wheel.steering_joint().name() &&
             wheel.steering_joint().limits().contains(steering_joint_state_.angle());
    }

    /**
     * @brief Get steering-joint state.
     * @return Const reference to steering-joint state.
     */
    const SteeringJointState& steering_joint_state() const
    {
      return steering_joint_state_;
    }

    private:
    SteeringJointState steering_joint_state_;  ///< Steering-joint state.
  };

  //////////////////////////////////////////////////////////////////////////////
  // COMMANDS
  //////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Generic joint command.
   */
  class JointCommand
  {
    public:
    /** @brief Build an empty joint command. */
    JointCommand() = default;

    /**
     * @brief Build a joint command.
     * @param joint_name Joint name.
     * @param value Command value (rad for steer, rad/s for rotation).
     * @throws std::invalid_argument If @p joint_name is empty.
     */
    JointCommand(const std::string& joint_name, const double value, const int64_t timestamp_ns):
      joint_name_{joint_name},
      value_{value},
      timestamp_ns_{timestamp_ns}
    {
      validate();
    }

    /**
     * @brief Get joint name.
     * @return Joint name.
     */
    const std::string& joint_name() const
    {
      return joint_name_;
    }

    /**
     * @brief Get command value.
     * @return Command value (rad or rad/s, depending on joint type).
     */
    double value() const
    {
      return value_;
    }

    /**
     * @brief Get command timestamp.
     * @return Timestamp associated with this joint command.
     */
    int64_t timestamp_ns() const
    {
      return timestamp_ns_;
    }

    private:
    void validate() const
    {
      detail::ensure_non_empty(joint_name_, "JointCommand.joint_name");
      detail::ensure_finite(value_, "JointCommand.value");
    }

    std::string joint_name_;   ///< Joint name.
    double value_{0.0};        ///< Command value (rad or rad/s, depending on joint type).
    int64_t timestamp_ns_{0};  ///< Timestamp associated with this joint command.
  };

  /** @brief Rotation-joint command specialization. */
  class RotationJointCommand: public JointCommand
  {
    public:
    /** @brief Build an empty rotation-joint command. */
    RotationJointCommand() = default;

    /**
     * @brief Build a rotation-joint command.
     * @param joint_name Rotation-joint name.
     * @param value Rotation command value (rad/s).
     */
    RotationJointCommand(const std::string& joint_name, const double value, const int64_t timestamp_ns):
      JointCommand(joint_name, value, timestamp_ns)
    {}
  };

  /** @brief Steering-joint command specialization. */
  class SteeringJointCommand: public JointCommand
  {
    public:
    /** @brief Build an empty steering-joint command. */
    SteeringJointCommand() = default;

    /**
     * @brief Build a steering-joint command.
     * @param joint_name Steering-joint name.
     * @param value Steering command value (rad).
     */
    SteeringJointCommand(const std::string& joint_name, const double value, const int64_t timestamp_ns):
      JointCommand(joint_name, value, timestamp_ns)
    {}
  };

  /**
   * @brief Command set for a wheel module.
   */
  class WheelCommand
  {
    public:
    /** @brief Build an empty wheel command. */
    WheelCommand() = default;

    /**
     * @brief Build a final wheel command from a desired wheel angular velocity.
     * @param wheel_ang_vel Desired wheel angular velocity (rad/s).
     * @param wheel Wheel model.
     * @param timestamp_ns Timestamp associated with this wheel command.
     */
    WheelCommand(const Wheel& wheel, const double wheel_ang_vel, const int64_t timestamp_ns = 0):
      wheel_name_{wheel.name()},
      rotation_joint_command_{
        wheel.rotation_joint().name(),
        std::clamp(wheel_ang_vel, wheel.rotation_joint().limits().lower(), wheel.rotation_joint().limits().upper()),
        timestamp_ns}
    {
      detail::ensure_non_empty(wheel_name_, "WheelCommand.wheel_name");
    }

    bool matches(const Wheel& wheel) const
    {
      return wheel_name_ == wheel.name() && rotation_joint_command_.joint_name() == wheel.rotation_joint().name() &&
             wheel.rotation_joint().limits().contains(rotation_joint_command_.value());
    }

    /**
     * @brief Get rotation-joint command.
     * @return Const reference to rotation-joint command.
     */
    const RotationJointCommand& rotation_joint_command() const
    {
      return rotation_joint_command_;
    }

    /**
     * @brief Get wheel name.
     * @return Wheel name.
     */
    const std::string& wheel_name() const
    {
      return wheel_name_;
    }

    protected:
    std::string wheel_name_;                       ///< Wheel name.
    RotationJointCommand rotation_joint_command_;  ///< Rotation-joint command.
  };

  /**
   * @brief Command set for a steerable wheel module.
   */
  class SteerableWheelCommand: public WheelCommand
  {
    public:
    /** @brief Build an empty steerable-wheel command. */
    SteerableWheelCommand() = default;

    /**
     * @brief Build a steerable-wheel command from a wheel kinematic solution.
     * @param wheel Steerable-wheel.
     * @param wheel_state Current steerable-wheel state used to select the best steering solution.
     * @param steering_angle Desired steering angle (rad).
     * @param wheel_ang_vel Desired wheel angular velocity (rad/s).
     * @param timestamp_ns Timestamp associated with this steerable-wheel command.
     */
    SteerableWheelCommand(const SteerableWheel& wheel,
                          const SteerableWheelState& wheel_state,
                          const double steering_angle,
                          const double wheel_ang_vel,
                          const int64_t timestamp_ns = 0):
      // The slice of the WheelCommand must be initialized, but we do not have at thist point
      // enough information to select the best steering solution. Therefore, we initialize it with
      // 0 ang_vel, i.e., the wheel is stopped. We will override those values at the end of this
      // constructor after selecting the best steering solution.
      WheelCommand(wheel, 0.0, timestamp_ns)
    {
      // If the wheel linear speed is almost zero, stop it and keep the steering angle currently reported by
      // the wheel state.
      constexpr double epsilon{0.0001};
      const double abs_wheel_vel{std::abs(wheel_ang_vel) * wheel.radius()};

      if(abs_wheel_vel < epsilon)
      {
        steering_joint_command_ = SteeringJointCommand{wheel.steering_joint().name(),
                                                       wheel_state.steering_joint_state().angle(),
                                                       timestamp_ns};

        return;
      };

      // A steerable wheel can often reach the same rolling direction with several equivalent angle/speed pairs.
      // Build those candidates first, then keep the one that best fits the current steering state and limits.
      const auto candidate_solutions{create_candidate_solutions(steering_angle, wheel_ang_vel)};
      const auto selected_solution{select_candidate_solution(candidate_solutions, wheel, wheel_state)};

      rotation_joint_command_ = RotationJointCommand{wheel.rotation_joint().name(),
                                                     std::clamp(selected_solution.wheel_ang_vel,
                                                                wheel.rotation_joint().limits().lower(),
                                                                wheel.rotation_joint().limits().upper()),
                                                     timestamp_ns};

      steering_joint_command_ = SteeringJointCommand{wheel.steering_joint().name(),
                                                     selected_solution.steering_angle,
                                                     timestamp_ns};
    }

    bool matches(const SteerableWheel& wheel) const
    {
      return WheelCommand::matches(wheel) && steering_joint_command_.joint_name() == wheel.steering_joint().name() &&
             wheel.steering_joint().limits().contains(steering_joint_command_.value());
    }

    /**
     * @brief Get steering-joint command.
     * @return Const reference to steering-joint command.
     */
    const SteeringJointCommand& steering_joint_command() const
    {
      return steering_joint_command_;
    }

    private:
    SteeringJointCommand steering_joint_command_;  ///< Steering-joint command.

    class CandidateSolution
    {
      public:
      double steering_angle{0.0};
      double wheel_ang_vel{0.0};
    };

    // Read the comments in the function `create_candidate_solutions` for the rationale behind this number.
    static constexpr std::size_t candidate_count_{5};

    /**
     * @brief Generate the equivalent steerable-wheel solutions associated with a kinematic solution.
     * @param steering_angle Base steering angle (rad).
     * @param wheel_ang_vel Base wheel angular velocity (rad/s).
     * @return Five equivalent steering/rotation candidate pairs.
     */
    static std::array<CandidateSolution, candidate_count_> create_candidate_solutions(const double steering_angle,
                                                                                      const double wheel_ang_vel)
    {
      // If steering_angle is a solution, then steering_angle + 2pi and steering_angle - 2pi are equivalent solutions
      // with the same wheel angular velocity, while steering_angle + pi and steering_angle - pi are equivalent
      // solutions with the opposite wheel angular velocity.
      // That leads to a total of 5 equivalent solutions, including the original one.
      constexpr double two_pi{2.0 * M_PI};

      return std::array<CandidateSolution, candidate_count_>{{
        CandidateSolution{steering_angle, wheel_ang_vel},
        CandidateSolution{steering_angle + two_pi, wheel_ang_vel},
        CandidateSolution{steering_angle - two_pi, wheel_ang_vel},
        CandidateSolution{steering_angle + M_PI, -wheel_ang_vel},
        CandidateSolution{steering_angle - M_PI, -wheel_ang_vel},
      }};
    }

    /**
     * @brief Compute the angular distance between a candidate steering angle and the current wheel steering state.
     * @param candidate_solution Candidate steering/rotation pair.
     * @param wheel_state Current steerable-wheel state.
     * @return Absolute angular distance in radians.
     */
    static double steering_distance_to_current(const CandidateSolution& candidate_solution,
                                               const SteerableWheelState& wheel_state)
    {
      return std::fabs(candidate_solution.steering_angle - wheel_state.steering_joint_state().angle());
    }

    /**
     * @brief Select the best candidate solution according to steering feasibility and continuity.
     * @param candidate_solutions Equivalent candidate solutions.
     * @param wheel Steerable-wheel model.
     * @param wheel_state Current steerable-wheel state.
     * @return Best candidate solution. Non-clamped solutions are preferred over clamped ones. Within each group,
     * the candidate with the smallest angular distance to the current wheel state is selected.
     */
    static CandidateSolution select_candidate_solution(
      const std::array<CandidateSolution, candidate_count_>& candidate_solutions,
      const SteerableWheel& wheel,
      const SteerableWheelState& wheel_state)
    {
      // Split the candidates into two groups. One group contains the candidates whose steering angle is already
      // within the limits. The other group contains the candidates that must be clamped. If there is at least
      // one non-clamped candidate, choose from that group. Otherwise, choose from the clamped group. In both
      // cases, select the candidate that is closest to the current steering angle.
      std::array<CandidateSolution, candidate_count_> non_clamped_solutions{};
      std::array<std::size_t, candidate_count_> non_clamped_indices{};
      std::size_t non_clamped_count{0};

      std::array<CandidateSolution, candidate_count_> clamped_solutions{};
      std::array<std::size_t, candidate_count_> clamped_indices{};
      std::size_t clamped_count{0};

      for(std::size_t i{0}; i < candidate_solutions.size(); ++i)
      {
        if(wheel.steering_joint().limits().contains(candidate_solutions[i].steering_angle))
        {
          // Keep candidates that already satisfy the steering limits unchanged.
          non_clamped_solutions[non_clamped_count] = candidate_solutions[i];
          non_clamped_indices[non_clamped_count]   = i;
          ++non_clamped_count;
        }
        else
        {
          // If the steering angle is outside the limits, project it onto the nearest allowed boundary and keep the
          // wheel angular velocity as-is. The selection step will decide later whether this clamped candidate is the
          // best fallback.
          clamped_solutions[clamped_count] = CandidateSolution{std::clamp(candidate_solutions[i].steering_angle,
                                                                          wheel.steering_joint().limits().lower(),
                                                                          wheel.steering_joint().limits().upper()),
                                                               candidate_solutions[i].wheel_ang_vel};
          clamped_indices[clamped_count]   = i;
          ++clamped_count;
        }
      }

      const CandidateSolution* chosen_solutions{nullptr};
      std::size_t chosen_count{0};

      if(non_clamped_count > 0)
      {
        // Prefer candidates that already satisfy the steering limits.
        chosen_solutions = non_clamped_solutions.data();
        chosen_count     = non_clamped_count;
      }
      else
      {
        // Only fall back to clamped candidates when every equivalent solution is outside the limits.
        chosen_solutions = clamped_solutions.data();
        chosen_count     = clamped_count;
      }

      std::size_t best_idx{0};
      double best_distance_to_current{steering_distance_to_current(chosen_solutions[0], wheel_state)};

      for(std::size_t i{1}; i < chosen_count; ++i)
      {
        const double distance_to_current{steering_distance_to_current(chosen_solutions[i], wheel_state)};

        if(distance_to_current < best_distance_to_current)
        {
          best_idx                 = i;
          best_distance_to_current = distance_to_current;
        }
      }

      return chosen_solutions[best_idx];
    }
  };

  //////////////////////////////////////////////////////////////////////////////
  // DESCRIPTORS
  //////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Aggregated wheel model, state, command and timestamps.
   */
  class WheelDescriptor
  {
    public:
    /**
     * @brief Build wheel descriptor from its model only.
     * @param wheel Wheel descriptor.
     */
    explicit WheelDescriptor(const Wheel& wheel): wheel_{wheel} {}

    /**
     * @brief Get wheel descriptor.
     * @return Const reference to wheel descriptor.
     */
    const Wheel& wheel() const
    {
      return wheel_;
    }

    /**
     * @brief Get wheel-state sample.
     * @return Const reference to wheel-state sample.
     */
    const WheelState& wheel_state() const
    {
      return wheel_state_;
    }

    /**
     * @brief Get wheel-command sample.
     * @return Const reference to wheel-command sample.
     */
    const WheelCommand& wheel_command() const
    {
      return wheel_command_;
    }

    /**
     * @brief Update wheel-command sample.
     * @param wheel_command New wheel-command sample.
     */
    void set_wheel_command(const WheelCommand& wheel_command)
    {
      wheel_command_ = wheel_command;
    }

    /**
     * @brief Update wheel-state sample.
     * @param wheel_state New wheel-state sample.
     */
    void set_wheel_state(const WheelState& wheel_state)
    {
      wheel_state_ = wheel_state;
    }

    private:
    Wheel wheel_;                 ///< Wheel kinematic descriptor.
    WheelState wheel_state_;      ///< Latest wheel state sample.
    WheelCommand wheel_command_;  ///< Latest wheel command sample.
  };

  /**
   * @brief Aggregated steerable-wheel model, state, command and timestamps.
   */
  class SteerableWheelDescriptor
  {
    public:
    /**
     * @brief Build steerable-wheel descriptor from its model only.
     * @param wheel Steerable-wheel descriptor.
     */
    explicit SteerableWheelDescriptor(const SteerableWheel& wheel): wheel_{wheel} {}

    /**
     * @brief Update steerable-wheel-command sample.
     * @param wheel_command New steerable-wheel-command sample.
     */
    void set_wheel_command(const SteerableWheelCommand& wheel_command)
    {
      wheel_command_ = wheel_command;
    }

    /**
     * @brief Update steerable-wheel-state sample.
     * @param wheel_state New steerable-wheel-state sample.
     */
    void set_wheel_state(const SteerableWheelState& wheel_state)
    {
      wheel_state_ = wheel_state;
    }

    /**
     * @brief Get steerable-wheel descriptor.
     * @return Const reference to steerable-wheel descriptor.
     */
    const SteerableWheel& wheel() const
    {
      return wheel_;
    }

    /**
     * @brief Get steerable-wheel-command sample.
     * @return Const reference to steerable-wheel-command sample.
     */
    const SteerableWheelCommand& wheel_command() const
    {
      return wheel_command_;
    }

    /**
     * @brief Get steerable-wheel-state sample.
     * @return Const reference to steerable-wheel-state sample.
     */
    const SteerableWheelState& wheel_state() const
    {
      return wheel_state_;
    }

    private:
    /**
     * @brief Configured kinematic model of this wheel module.
     */
    SteerableWheel wheel_;

    /**
     * @brief Most recently stored command for this wheel module.
     */
    SteerableWheelCommand wheel_command_;

    /**
     * @brief Most recently stored runtime state for this wheel module.
     */
    SteerableWheelState wheel_state_;
  };

}  // namespace ground_vehicle_kinematics
