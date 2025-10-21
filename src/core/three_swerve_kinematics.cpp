#include "ground_vehicle_kinematics/core/three_swerve_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>
#include <angles/angles.h>

#include "ground_vehicle_kinematics/core/utils.hpp"

namespace ground_vehicle_kinematics
{

  /*
   * ============================================================================
   * FRAME ALIGNMENT ASSUMPTIONS FOR SWERVE DRIVE KINEMATICS
   * ============================================================================
   *
   * This solver assumes a specific kinematic chain configuration that is standard
   * for swerve drive architectures:
   *
   * KINEMATIC CHAIN:
   * steerable_joint:
   *     robot_root_link (e.g base_link) → steerable_link
   * rotation_joint:
   *     steerable_link → wheel_link
   *
   * CRITICAL FRAME ALIGNMENT REQUIREMENTS:
   *
   * 1. STEERABLE JOINT CONFIGURATION:
   *    - steerable_joint connects parent_link (usually base_link) to steerable_link
   *    - steerable_link frame MUST be aligned with parent_link frame
   *    - Translation: free positioning in x, y, z (specifies wheel module location)
   *    - Rotation: MUST be identity (roll = pitch = yaw = 0)
   *    - This means steerable_link x,y,z axes are parallel to parent_link axes
   *
   * 2. ROTATION JOINT CONFIGURATION:
   *    - rotation_joint connects steerable_link to wheel_link
   *    - wheel_link frame MUST be aligned with steerable_link frame
   *    - Translation: only z offset allowed (y = y = 0, z = free for wheel vertical offset)
   *    - Rotation: MUST be identity (roll = pitch = yaw = 0)
   *    - This ensures wheel rotation axis is always vertical (parallel to z)
   *
   * MATHEMATICAL CONSEQUENCE:
   * - Wheel position (x_i,y_i) relative to base_link equals steerable_joint origin
   * - No coordinate transformations needed between frames
   * - Kinematic equations directly use steerable_joint.origin.{x,y} values
   *
   * TYPICAL URDF EXAMPLE:
   * <joint name="steerable_joint_front_left" type="revolute">
   *   <parent link="base_link"/>
   *   <child link="steerable_link_front_left"/>
   *   <origin xyz="0.35 0.25 0.0" rpy="0 0 0"/>  <- ALIGNED FRAMES
   * </joint>
   *
   * <joint name="rotation_joint_front_left" type="continuous">
   *   <parent link="steerable_link_front_left"/>
   *   <child link="wheel_link_front_left"/>
   *   <origin xyz="0 0 -0.05" rpy="0 0 0"/>      <- ALIGNED FRAMES
   * </joint>
   *
   * This configuration is natural for most swerve drive and Ackermann architectures and ensures the kinematic equations
   * remain in the simple 2D form documented in the mathematical derivation.
   */

  std::optional<Twist> ThreeSwerveKinematicsSolver::solve_direct(const std::array<SteerableWheel, 3>& sws,
                                                                 const std::array<SteerableWheelState, 3>& sw_states)
  {
    // Build the overdetermined least-squares system A * z ≈ b, where z = [vx, vy, wz]^T
    // For each wheel i:
    //   v_ix = (R_i * alpha_i) * cos(theta_i)
    //   v_iy = (R_i * alpha_i) * sin(theta_i)
    // and the rigid-body relation at the contact point is:
    //   [1 0 -y_i] [vx]   [v_ix]
    //   [0 1  x_i] [vy] = [v_iy]
    Eigen::Matrix<double, 6, 3> A;
    Eigen::Matrix<double, 6, 1> b;

    for(size_t i{0}; i < sw_states.size(); ++i)
    {
      const auto& sw       = sws[i];
      const auto& sw_state = sw_states[i];

      // Ensure correspondence by index between sw and sw_state
      const auto theta{sw_state.steerable_joint_state.theta};  // θᵢ
      const auto x{sw.steerable_joint.origin.x};               // xᵢ
      const auto y{sw.steerable_joint.origin.y};               // yᵢ
      const auto alpha{sw_state.rotation_joint_state.alpha};   // αᵢ
      const auto R{sw.radius};                                 // Rᵢ

      // Validate numeric inputs per wheel
      if(!std::isfinite(theta) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(alpha) ||
         !std::isfinite(R) || (R <= 0.0))
      {
        std::cerr << "[WARN] solve_direct: invalid input for wheel '" << sw.name << "' (idx=" << i << ")"
                  << ", theta=" << theta << ", x=" << x << ", y=" << y << ", alpha=" << alpha << ", R=" << R
                  << std::endl;

        return std::nullopt;
      }

      // sᵢ = Rᵢ x αᵢ (linear speed at wheel contact point is radius times angular velocity)
      const auto s{R * alpha};
      const auto vix{s * std::cos(theta)};
      const auto viy{s * std::sin(theta)};

      // Fill rows 2*i and 2*i+1 in A and b
      A(2 * i, 0) = 1.0;
      A(2 * i, 1) = 0.0;
      A(2 * i, 2) = -y;

      A(2 * i + 1, 0) = 0.0;
      A(2 * i + 1, 1) = 1.0;
      A(2 * i + 1, 2) = x;

      b(2 * i)     = vix;
      b(2 * i + 1) = viy;
    }

    // Solve LS using QR; if rank-deficient, fall back to SVD
    Eigen::Vector3d z;
    Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 6, 3>> qr(A);

    if(qr.rank() == 3)
    {
      z = qr.solve(b);
    }
    else
    {
      Eigen::JacobiSVD<Eigen::Matrix<double, 6, 3>> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
      z = svd.solve(b);
    }

    // Package the result as a Twist message
    Twist twist;
    twist.vx = z(0);
    twist.vy = z(1);
    twist.wz = z(2);

    // Ground vehicle: these components are unused/undefined
    twist.vz = std::numeric_limits<double>::quiet_NaN();
    twist.wx = std::numeric_limits<double>::quiet_NaN();
    twist.wy = std::numeric_limits<double>::quiet_NaN();

    return twist;
  }

  std::array<SteerableWheelState, 3> ThreeSwerveKinematicsSolver::solve_inverse(
    const std::array<SteerableWheel, 3>& sws,
    const Twist& twist,
    const std::array<SteerableWheelState, 3>& prev_sw_states)
  {
    std::array<SteerableWheelState, 3> sw_states{};

    // Process each wheel independently using the inverse kinematics algorithm.
    // 1. Calculate local velocity at wheel i contact point: 𝐯_i = 𝐯 + 𝛚 × 𝐫_i = [vx - ω * y_i, vy + ω * x_i]
    // 2. Compute steering direction for the wheel i: θ_i = atan2(v_iy, v_ix)
    // 3. Compute rolling speed for wheel i: ||v_i||
    // 4. Compute wheel angular velocity: α_i = ||v_i||/wheel_radius

    for(size_t i{0}; i < sws.size(); ++i)
    {
      const auto& sw            = sws[i];
      const auto& prev_sw_state = prev_sw_states[i];

      // CRITICAL: 'sw', 'prev_sw_state', and 'result' must all correspond to the same physical wheel.
      // The ROS wrapper ensures this correlation by consistent indexing
      const auto x{sw.steerable_joint.origin.x};
      const auto y{sw.steerable_joint.origin.y};  // y position for wheel i

      // STEP 1: Compute contact point velocity using rigid body kinematics
      // General equation: 𝐯_i = 𝐯 + 𝛚 × 𝐫_i = [vx - ω * y_i, vy + ω * x_i]
      const Eigen::Vector2d v{twist.vx - twist.wz * y, twist.vy + twist.wz * x};

      // STEP 2: Compute desired steering angle from velocity direction.
      // θ_i = atan2(v_iy, v_ix) (direction wheel should point to roll without slipping)
      SteerableWheelState sw_state_a;  // _a, implies, there is a candidate _b next, and a selection process
      sw_state_a.wheel_name                       = sw.name;
      sw_state_a.rotation_joint_state.joint_name  = sw.rotation_joint.name;
      sw_state_a.steerable_joint_state.joint_name = sw.steerable_joint.name;

      sw_state_a.steerable_joint_state.theta = angles::normalize_angle(std::atan2(v.y(), v.x()));

      // STEP 3: Compute required rolling speed
      // s_i = ||vᵢ|| (magnitude of contact point velocity)
      const auto speed{v.norm()};

      // STEP 4: Convert rolling speed to wheel angular velocity
      // αᵢ = s_ᵢ/wheel_radius (wheel angular velocity to achieve rolling speed)
      sw_state_a.rotation_joint_state.alpha = (sw.radius > 0.0) ? (speed / sw.radius) : 0.0;

      // DIRECTIONAL AMBIGUITY RESOLUTION:
      //
      // At this point we have calculated the "natural" steering direction (base_theta) and
      // rolling speed (alpha) needed to achieve the desired velocity vector 𝐯_i.
      //
      // However, there's a fundamental ambiguity in swerve drive kinematics:
      // The SAME velocity vector 𝐯_i can be achieved by TWO different wheel configurations:
      //
      // Configuration A (Natural):
      //   - Wheel points in direction 'base_theta'
      //   - Wheel rolls forward with angular velocity 'alpha'
      //
      // Configuration B (Flipped):
      //   - Wheel points in opposite direction 'base_theta + pi'
      //   - Wheel rolls backward with angular velocity '-alpha'
      //
      // Both configurations produce identical contact point velocity 𝐯_i!
      //
      //
      // WHY WE NEED PREVIOUS STATE (prev_sw_states)?
      //
      // Without continuity information, the solver might constantly switch between equivalent configurations, causing:
      // - Sudden 180° steering jumps between control cycles
      // - Excessive wear on steering actuators
      // - Jerky robot motion and instability
      // - Wasted energy from unnecessary large steering motions
      //
      // SELECTION STRATEGY:
      //
      // We must choose between these configurations considering:
      // 1. MECHANICAL LIMITS: Does the steering angle fit within joint limits?
      // 2. CONTINUITY: Which option minimizes steering angle change from PREVIOUS STATE?
      // 3. SATURATION HANDLING: What to do if neither option fits within limits?
      //
      // FLIP-AND-REVERSE STRATEGY:
      //
      // This strategy evaluates both candidates and selects the optimal one:
      // - FLIP: Rotate steering direction by 180° (add pi to angle)
      // - REVERSE: Invert wheel rotation direction (negate angular velocity)
      // - CHOOSE: Pick candidate that respects limits AND minimizes steering motion relative to prev_sw_states
      // - FALLBACK: If both violate limits, saturate the best option to nearest boundary
      //
      // This approach is essential for:
      // - Avoiding unnecessary 180° steering rotations that waste time and energy
      // - Respecting physical constraints of steering actuators
      // - Maintaining smooth robot motion without steering discontinuities
      // - Preventing "thrashing" between equivalent configurations

      // Select best candidate considering limits and continuity
      SteerableWheelState sw_state_b;
      sw_state_b.wheel_name                       = sw.name;
      sw_state_b.rotation_joint_state.joint_name  = sw.rotation_joint.name;
      sw_state_b.steerable_joint_state.joint_name = sw.steerable_joint.name;
      sw_state_b.steerable_joint_state.theta = angles::normalize_angle(sw_state_a.steerable_joint_state.theta + M_PI);
      sw_state_b.rotation_joint_state.alpha  = -sw_state_a.rotation_joint_state.alpha;


      sw_states[i] = select_best_sw_state(sw, prev_sw_state, sw_state_a, sw_state_b);
    }

    return sw_states;
  }

  //////////////////////////////////////////////////////////////////////////////
  // PRIVATE HELPER METHODS
  //////////////////////////////////////////////////////////////////////////////

  bool ThreeSwerveKinematicsSolver::is_within_steering_limits(double angle, double lower_limit, double upper_limit)
  {
    // Since we are comparing floating point numbers, we cannot rely on exact equality (==)
    // The way to 'achive the equality with the limits of the interval' is to use a small tolerance value
    // to extend the interval just a little bit, so now by using > and < we can include the equality case as well.
    constexpr double tolerance{1e-9};

    return (angle > (lower_limit - tolerance)) && (angle < (upper_limit + tolerance));
  }

  //////////////////////////////////////////////////////////////////////////////

  double ThreeSwerveKinematicsSolver::compute_angular_distance_to_angle(double target_angle, double reference_angle)
  {
    return std::fabs(angles::normalize_angle(target_angle - reference_angle));
  }

  //////////////////////////////////////////////////////////////////////////////

  SteerableWheelState ThreeSwerveKinematicsSolver::select_best_sw_state(const SteerableWheel& sw,
                                                                        const SteerableWheelState& prev_sw_state,
                                                                        const SteerableWheelState& sw_state_a,
                                                                        const SteerableWheelState& sw_state_b)
  {
    /*
     * SELECTION LOGIC FOR STEERABLE WHEEL STATES
     * ==========================================
     *
     * Given two mathematically equivalent candidates (sw_state_a, sw_state_b) that produce
     * the same wheel contact velocity vector, select the optimal one using this priority:
     *
     * PHASE 1: Mechanical Limits Compliance
     * - Priority 1a: If both candidates are within steering limits -> choose minimum angular distance to previous state
     * - Priority 1b: If only candidate A is within limits -> return sw_state_a
     * - Priority 1c: If only candidate B is within limits -> return sw_state_b
     *
     * PHASE 2: Saturation Fallback (both candidates violate limits)
     * - Choose the candidate with minimum angular displacement from previous state
     * - Saturate (clamp) the steering angle to the nearest mechanical limit
     * - Mark steering_saturated = true for diagnostic purposes
     * - Return the best feasible solution (graceful degradation)
     */

    // Extract steering angles from candidate states
    const auto theta_a{sw_state_a.steerable_joint_state.theta};
    const auto theta_b{sw_state_b.steerable_joint_state.theta};

    // Determine if we have valid previous state
    const auto prev_theta{!std::isnan(prev_sw_state.steerable_joint_state.theta) ?
                            angles::normalize_angle(prev_sw_state.steerable_joint_state.theta) :
                            0.0};

    // PHASE 1: Try to find a candidate that satisfies mechanical limits
    const auto theta_a_within_limits{is_within_steering_limits(theta_a,
                                                               sw.steerable_joint.angular_limits.lower,
                                                               sw.steerable_joint.angular_limits.upper)};

    const auto theta_b_within_limits{is_within_steering_limits(theta_b,
                                                               sw.steerable_joint.angular_limits.lower,
                                                               sw.steerable_joint.angular_limits.upper)};

    // Priority: mechanical limits compliance, then minimize angular displacement
    if(theta_a_within_limits && theta_b_within_limits)
    {
      // Both candidates satisfy limits: choose the one with minimum angular displacement
      const auto angular_distance_a{compute_angular_distance_to_angle(theta_a, prev_theta)};
      const auto angular_distance_b{compute_angular_distance_to_angle(theta_b, prev_theta)};

      return (angular_distance_a <= angular_distance_b) ? sw_state_a : sw_state_b;
    }
    else if(theta_a_within_limits)
    {
      // Only candidate A satisfies limits
      return sw_state_a;
    }
    else if(theta_b_within_limits)
    {
      // Only candidate B satisfies limits
      return sw_state_b;
    }
    else
    {
      // PHASE 2: Handle limit violations with saturation (fallback strategy)
      // Neither candidate satisfies mechanical limits: choose best option and saturate
      // This handles extreme maneuvers where desired wheel direction exceeds physical capabilities
      const auto angular_distance_a{compute_angular_distance_to_angle(theta_a, prev_theta)};
      const auto angular_distance_b{compute_angular_distance_to_angle(theta_b, prev_theta)};

      // Choose candidate with minimum angular displacement from previous state
      SteerableWheelState chosen_sw_state{(angular_distance_a <= angular_distance_b) ? sw_state_a : sw_state_b};

      // Saturate steering angle to nearest valid limit
      chosen_sw_state.steerable_joint_state.theta              = std::clamp(chosen_sw_state.steerable_joint_state.theta,
                                                               sw.steerable_joint.angular_limits.lower,
                                                               sw.steerable_joint.angular_limits.upper);
      chosen_sw_state.steerable_joint_state.steering_saturated = true;  // Flag for logging/monitoring

      return chosen_sw_state;
    }
  }

}  // namespace ground_vehicle_kinematics
