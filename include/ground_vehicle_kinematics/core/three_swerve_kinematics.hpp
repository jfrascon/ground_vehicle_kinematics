#pragma once

#include <array>
#include <optional>

#include <Eigen/Dense>

#include "ground_vehicle_kinematics/core/types.hpp"

namespace ground_vehicle_kinematics
{

  /// ThreeSwerveKinematicsSolver provides direct and inverse kinematics for a 3-module swerve drive.
  class ThreeSwerveKinematicsSolver
  {
    public:
    // Compute the chassis twist given the steerable wheel states (angular velocities for rotation joints and
    // angle for steerable joints).
    // Use of reference_wrapper because std::array does not allow to use references directly.
    static std::optional<Twist> solve_direct(
      const std::array<std::reference_wrapper<const SteerableWheel>, 3>& st_wheels,
      const std::array<std::reference_wrapper<const SteerableWheelState>, 3>& st_wheel_states);

    static void compute_health_dk_solution(const Eigen::Matrix3d& Cp,
                                           const Eigen::Matrix3d& Cn,
                                           const Eigen::Vector3d& bp,
                                           double max_s_value,
                                           double min_s_value_kept,
                                           double rank_kept,
                                           const Eigen::Vector3d& twist_vector);

    // Compute wheel orientation and velocity to achieve the provided twist.
    // Use of reference_wrapper because std::array does not allow to use references directly.
    static std::array<SteerableWheelCommand, 3> solve_inverse(
      const std::array<std::reference_wrapper<const SteerableWheel>, 3>& st_wheels,
      const std::array<std::reference_wrapper<const SteerableWheelState>, 3>& st_wheel_states,
      const Twist& twist,
      const std::array<SteerableWheelCommand, 3>& prev_st_wheel_commands);

    private:
    // Helper functions for kinematics calculations

    // Check if steering angle is within mechanical limits
    static bool is_within_steering_limits(double angle, const Limits& limits);

    // Select best candidate considering limits and continuity
    static SteerableWheelCommand select_best_sw_state(const SteerableWheel& st_wheel,
                                                      const SteerableWheelState& st_wheel_state,
                                                      const SteerableWheelCommand& st_wheel_command_a,
                                                      const SteerableWheelCommand& st_wheel_command_b);
  };

}  // namespace ground_vehicle_kinematics
