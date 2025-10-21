#pragma once

#include <array>
#include <optional>

#include "ground_vehicle_kinematics/core/types.hpp"

namespace ground_vehicle_kinematics
{

  /// ThreeSwerveKinematicsSolver provides direct and inverse kinematics for a 3-module swerve drive.
  class ThreeSwerveKinematicsSolver
  {
    public:
    /// Compute the chassis twist that would generate the given wheel states.
    static std::optional<Twist> solve_direct(const std::array<SteerableWheel, 3>& sw,
                                             const std::array<SteerableWheelState, 3>& sw_states);

    /// Compute wheel steering + rotation states to achieve the commanded twist.
    static std::array<SteerableWheelState, 3> solve_inverse(const std::array<SteerableWheel, 3>& sw,
                                                            const Twist& twist,
                                                            const std::array<SteerableWheelState, 3>& prev_sw_states);

    private:
    // Helper functions for kinematics calculations

    // Compute angular distance from reference angle to target angle
    static double compute_angular_distance_to_angle(double target_angle, double reference_angle);

    // Check if steering angle is within mechanical limits
    static bool is_within_steering_limits(double angle, double lower_limit, double upper_limit);

    // Select best candidate considering limits and continuity
    static SteerableWheelState select_best_sw_state(const SteerableWheel& sw,
                                                    const SteerableWheelState& prev_sw_state,
                                                    const SteerableWheelState& sw_state_a,
                                                    const SteerableWheelState& sw_state_b);
  };

}  // namespace ground_vehicle_kinematics
