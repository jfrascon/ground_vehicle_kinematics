#include "ground_vehicle_kinematics/core/three_swerve_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
#include <stdexcept>
#include <utility>

#include <Eigen/SVD>
#include <angles/angles.h>

namespace ground_vehicle_kinematics
{
  /**
  THIS SOLVER MAKES SOME ASSUMPTIONS ABOUT THE STEERABLE WHEEL MODULES FOR THE KINEMATICS CALCULATIONS TO BE VALID.

  Each steerable wheel has two joints:
  1. 'steerable_joint': 'robot_root_link' (e.g base_link) is the parent link of 'steerable_link'.
  2. 'rotation_joint': 'steerable_link' is the parent link of 'wheel_link'.

  In short, for each steerable wheel: 'robot_root_link' -> 'steerable_link' -> 'wheel_link'

  Axis conventions for 'wheel_link':
  - The Y-axis of 'wheel_link' is the physical rotation axis of the wheel.
  - The X-axis of 'wheel_link' is the rolling direction of the wheel.
  - Consequently, the Z-axis of 'wheel_link' is normal to the ground plane

  Alignment of the 'steerable_link' and 'wheel_link':
  'wheel_link' is aligned with 'steerable_link'. That is, the pose of 'wheel_link' relative to 'steerable_link' is:
  position: (X = 0, Y = 0, Z = FREE), orientation: (ROLL = 0, PITCH = 0, YAW = 0)
  The Z = FREE allows a vertical offset between 'steerable_link' and 'wheel_link' if necessary.

  With this alignment, the X/Y/Z axes of 'wheel_link' coincide with those of 'steerable_link', so for all kinematic
  formulas below you only need to reason in the 'steerable_link' frame of each module:
  X = rolling direction, Y = wheel rotation axis, Z = up.

  Placement of the three modules relative to the robot frame (2D, top view)
  --------------------------------------------------------------------------
  We use:
  l0 > 0 : distance from the robot-frame origin O to wheel #0 (the "single").
  l  > 0 : common distance to wheels #1 and #2 (the pair), so l1 = l2 = l.
  θi     : steer angle of wheel i, measured from the robot-frame +x-axis.
  αi     : polar angle of the position of wheel i, defined by αi = atan2(yi, xi).

  Wheel #0 ("st_wheel_0", the single) lies on the x-axis of the robot's frame, on either side:
  α0 = 0 -> (x0, y0) = ( l0 * cos(α0), l0 * sin(α0) ) = ( +l0, 0 )
  α0 = π -> (x0, y0) = ( l0 * cos(α0), l0 * sin(α0) ) = ( -l0, 0 )

  Wheels #1 and #2 ("st_wheel_1" and "st_wheel_2", the pair) lie in the half-plane opposite to the side where
  'st_wheel_0' is placed, and they are symmetric with respect to the x-axis of the robot frame:

  If 'st_wheel_0' lies on the positive x-axis of the robot frame, then 'st_wheel_1' and 'st_wheel_2' lie in the
  half-plane with negative x (x < 0).
  Using polar angles from the +x-axis:
    st_wheel_1 (y > 0): α1 ∈ [π/2, π),         (x1, y1) = ( l * cos(α1),  l * sin(α1) )
    st_wheel_2 (y < 0): α2 = -α1 ∈ (-π, -π/2], (x2, y2) = ( l * cos(α2),  l * sin(α2) ) =
                                                         = ( l * cos(-α1), l * sin(-α1) ) =
                                                         = ( l * cos(α1), -l * sin(α1) ) =

  If 'st_wheel_0' lies on the negative x-axis of the robot frame, then 'st_wheel_1' and 'st_wheel_2' lie in the
  half-plane with positive x (x > 0).
  Using polar angles from the +x-axis:
    st_wheel_1 (y > 0): α1 ∈ (0, π/2],        (x1, y1) = ( l * cos(α1),  l * sin(α1) )
    st_wheel_2 (y < 0): α2 = -α1 ∈ [-π/2, 0), (x2, y2) = ( l * cos(α2),  l * sin(α2) ) =
                                                        = ( l * cos(-α1), l * sin(-α1) ) =
                                                        = ( l * cos(α1), -l * sin(α1) )

  In all cases, the pair preserves symmetry about the robot x-axis:
  α2 = -α1 and l1 = l2 = l.

  If l0 = l1 = l2 = l, the formulation still holds with no special treatment.

  It is also important to meention that for each steerable wheel, the frame of the 'steerable_link' MUST BE
  ALIGNED with the frame of the 'robot_root_link' when at rest; i.e, the positive x-axis of each steerable_link points
  to the same direction as the positive x-axis of the robot_root_link, the positive y-axis of each steerable_link points
  to the same direction as the positive y-axis of the robot_root_link, and the positive z-axis of each steerable_link
  points to the same direction as the positive z-axis of the robot.

  If you want to have a complete understanding of the frame alignment assumptions, and all the details involving the
  computations of the kinematics equations for the three swerve drive configuration, you can read the pdf document on
  the 'doc' folder.

  Finally, each wheel is considered to have the same radius 'r'.

  Example of a fragment of an URDF file defining the joints associated to the steerable wheels:

  <joint name="st_wheel_0_steerable_joint" type="revolute">
    <parent link="base_link"/>
    <child link="st_wheel_0_steerable_link"/>
    <origin rpy="0 0 0" xyz="0.46 0 0.067"/>
    <axis xyz="0 0 1"/>
    <limit effort="100000" lower="-3.14159265" upper="3.14159265" velocity="100"/>
  </joint>

  <joint name="st_wheel_0_rotation_joint" type="continuous">
    <parent link="st_wheel_0_steerable_link"/>
    <child link="st_wheel_0_rotation_link"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="100000" velocity="100"/>
  </joint>

  <joint name="st_wheel_1_steerable_joint" type="revolute">
    <parent link="base_link"/>
    <child link="st_wheel_1_steerable_link"/>
    <origin rpy="0 0 0" xyz="-0.46 0.219 0.067"/>
    <axis xyz="0 0 1"/>
    <limit effort="100000" lower="-3.14159265" upper="3.14159265" velocity="100"/>
  </joint>

  <joint name="st_wheel_1_rotation_joint" type="continuous">
    <parent link="st_wheel_1_steerable_link"/>
    <child link="st_wheel_1_rotation_link"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="100000" velocity="100"/>
  </joint>

  <joint name="st_wheel_2_steerable_joint" type="revolute">
    <parent link="base_link"/>
    <child link="st_wheel_2_steerable_link"/>
    <origin rpy="0 0 0" xyz="-0.46 -0.219 0.067"/>
    <axis xyz="0 0 1"/>
    <limit effort="100000" lower="-3.14159265" upper="3.14159265" velocity="100"/>
  </joint>

  <joint name="st_wheel_2_rotation_joint" type="continuous">
    <parent link="st_wheel_2_steerable_link"/>
    <child link="st_wheel_2_rotation_link"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="100000" velocity="100"/>
  </joint>

  This is the kinematic model for the three swerve drive configuration:

  Cp = [  cos(θ0)   sin(θ0)    l0 * sin(θ0 - α0) ]
       [  cos(θ1)   sin(θ1)    l1 * sin(θ1 - α1) ]
       [  cos(θ2)   sin(θ2)    l2 * sin(θ2 - α2) ]
                         3x3

  Cn = [ -sin(θ0)   cos(θ0)    l0 * cos(θ0 - α0) ]
       [ -sin(θ1)   cos(θ1)    l1 * cos(θ1 - α1) ]
       [ -sin(θ2)   cos(θ2)    l2 * cos(θ2 - α2) ]
                         3x3

  [  Cp ] *  [ vx ]   [ r * 𝛾0 ]
  [  Cn ]    [ vy ] = [ r * 𝛾1 ]
             [ wz ]   [ r * 𝛾2 ]
                      [   0    ]
                      [   0    ]
                      [   0    ]
   6x3         3x1       6x1

  */

  std::optional<Twist> ThreeSwerveKinematicsSolver::solve_direct(
    const std::array<std::reference_wrapper<const SteerableWheel>, 3>& st_wheels,
    const std::array<std::reference_wrapper<const SteerableWheelState>, 3>& st_wheel_states)
  {
    // Why we solve the stacked system with SVD
    // ----------------------------------------
    //
    // We build a 6x3 linear system C * twist_vector ≈ b by stacking two blocks:
    //   Cp (3x3): rolling compatibility along the wheel tangent.
    //   Cn (3x3): lateral no-slip (normal direction).
    // So C = [Cp; Cn] and b = [r * gamma_0; r * gamma_1; r * gamma_2; 0; 0; 0] = r * gamma.
    // The unknown is the platform twist_vector = [vx, vy, wz]^T.
    // We compute twist_vector as the Least-Squares (LS) solution: twist_vector* = argmin || C * twist_vector - b ||^2
    // This means: among all twists, pick the one that best fits the measured wheel speeds (top block) while keeping
    // lateral slip as close to zero as possible (bottom block).
    //
    // Why not invert Cp directly?
    // ---------------------------
    // Cp alone is 3x3, so in theory you could solve Cp * twist_vector = r * gamma.
    // In practice Cp becomes singular or very ill-conditioned in common steer configurations (e.g., three modules
    // parallel for straight motion, or one module radial).
    // Then 'inverting' Cp amplifies measurement noise dramatically and may even fail.
    // Stacking C = [Cp; Cn] adds lateral constraints that usually restore full rank and reduce sensitivity to those
    // bad geometries.
    //
    // Why SVD for the LS solve?
    // -------------------------
    // The SVD decomposes C as C = U * S * V^T, where S contains the singular values s_i ≥ 0.
    // Each s_i measures how much information the data provide in a particular direction of vector twist.
    // If s_i is tiny, that direction is poorly observed: forcing the fit along it only injects noise.
    //
    // We therefore:
    // 1) Truncate tiny singular values (drop s_i ≤ rel_thr * s_max):
    //    Ignore directions that the data cannot see reliably.
    // 2) Damp the kept ones with Tikhonov: use s / (s^2 + lambda) instead of 1/s.
    //    This limits the effect of near-degenerate directions and prevents large coefficients when Cp and Cn are
    //    slightly inconsistent (sensor noise, timing jitter, mild slip).
    //
    // Net effect: stable estimates even at 'awkward' steer angles, with negligible CPU cost for a 6x3 system.
    //
    // Could we do something simpler?
    // ------------------------------
    // - QR on Cp only: very fast and fine *when* Cp is well-conditioned, but it fails exactly when you most need
    //   robustness (parallel/radial cases).
    // - Normal equations (C^T C): algebraically fine but numerically worse than SVD because it squares the condition
    //   number.
    // - QR on the stacked C: better than Cp-only, but still less robust than SVD near rank-deficient situations, and
    //   it lacks an explicit, principled way to truncate/damp weak directions.
    //
    // Tuning and sanity checks
    // ------------------------
    // - rel_thr (default 1e-6): how aggressively we drop weak directions.
    //   Larger  -> more conservative (drop more).
    //   Smaller -> keep more, but risk noise.
    // - lambda (default 1e-4): how much we damp the kept directions.
    //   Larger  -> smoother/safer, but slightly biased.
    //   Smaller -> crisper, but riskier.
    // - Diagnostics you can log per cycle:
    //     * singular values S
    //     * a 'kept' condition number kappa = s_max / min_kept_s
    //     * residuals:   rp = Cp*twist_vector - r*gamma   (rolling fit)
    //                    rn = Cn*twist_vector             (lateral no-slip)
    //   If kappa blows up or residuals grow, you are in a weak geometry or your measurements are inconsistent
    //   (latency, sync, or slip). The SVD will still give a controlled answer, but the logs tell you *why*.
    //
    // Bottom line
    // -----------
    // The stacked LS + SVD (with truncation and damping) is the smallest, clearest, and most robust method that:
    // - respects the physics (rolling + no-slip),
    // - remains numerically stable at problematic steers,
    // - and is trivial to maintain for a 6x3 system running at 50–1000 Hz.

    // Build Cp and Cn given θi, αi and li.
    //   Cp(i,:) = [  cos(θ_i)   sin(θ_i)    l_i * sin(θ_i - α_i) ]
    //   Cn(i,:) = [ -sin(θ_i)   cos(θ_i)    l_i * cos(θ_i - α_i) ]
    //
    // We do it this way because:
    // - First two columns encode the tangent/normal directions induced by steer.
    // - Third column captures the lever-arm coupling from chassis yaw rate to the  wheel contact line (sine/cosine of
    //   θ_i - α_i times l_i).

    // Create Cp and Cn.
    // Cp = [ cos(θ0)  sin(θ0)  l0 * sin(θ0 - α0) ]
    //      [ cos(θ1)  sin(θ1)  l1 * sin(θ1 - α1) ]
    //      [ cos(θ2)  sin(θ2)  l2 * sin(θ2 - α2) ]
    //                        3x3

    // Cn = [ -sin(θ0)  cos(θ0)  l0 * cos(θ0 - α0) ]
    //      [ -sin(θ1)  cos(θ1)  l1 * cos(θ1 - α1) ]
    //      [ -sin(θ2)  cos(θ2)  l2 * cos(θ2 - α2) ]
    //                        3x3

    // bp = [ r * 𝛾0 ]
    //      [ r * 𝛾1 ]
    //      [ r * 𝛾2 ]
    //         3x1

    // bn = [ 0 ]
    //      [ 0 ]
    //      [ 0 ]
    //       3x1

    Eigen::Matrix3d Cp;
    Eigen::Matrix3d Cn;
    Eigen::Vector3d bp;
    Eigen::Vector3d bn{0.0, 0.0, 0.0};  // Ideal lateral no-slip (zero). Model prior that stabilizes.

    for(size_t i{0}; i < 3UL; ++i)
    {
      const auto& st_wheel       = st_wheels[i].get();
      const auto& st_wheel_state = st_wheel_states[i].get();

      // αᵢ
      const auto alpha_i{std::atan2(st_wheel.steerable_joint.origin.y, st_wheel.steerable_joint.origin.x)};

      // l_i
      const auto l_i{std::hypot(st_wheel.steerable_joint.origin.x, st_wheel.steerable_joint.origin.y)};

      // θ_i
      const auto theta_i{st_wheel_state.steerable_joint_state.angle};

      // θᵢ - αᵢ
      const auto angle_diff{theta_i - alpha_i};

      // cos(θᵢ) and sin(θᵢ)
      const auto cos_theta_i{std::cos(theta_i)};
      const auto sin_theta_i{std::sin(theta_i)};

      Cp(i, 0) = cos_theta_i;
      Cp(i, 1) = sin_theta_i;
      Cp(i, 2) = l_i * std::sin(angle_diff);

      Cn(i, 0) = -sin_theta_i;
      Cn(i, 1) = cos_theta_i;
      Cn(i, 2) = l_i * std::cos(angle_diff);

      bp(i) = st_wheel.radius * st_wheel_state.rotation_joint_state.ang_vel;
    }

    // Assemble the 6x3 stacked matrix C and the 6x1 right-hand side b.
    // C = [Cp] = [  cos(θ0)  sin(θ0)  l0 * sin(θ0 - α0) ]
    //     [Cn]   [  cos(θ1)  sin(θ1)  l1 * sin(θ1 - α1) ]
    //            [  cos(θ2)  sin(θ2)  l2 * sin(θ2 - α2) ]
    //            [ -sin(θ0)  cos(θ0)  l0 * cos(θ0 - α0) ]
    //            [ -sin(θ1)  cos(θ1)  l1 * cos(θ1 - α1) ]
    //            [ -sin(θ2)  cos(θ2)  l2 * cos(θ2 - α2) ]

    Eigen::Matrix<double, 6, 3> C;
    C << Cp, Cn;

    // b = [bp] = [ r * 𝛾0 ]
    //     [bn] = [ r * 𝛾1 ]
    //            [ r * 𝛾2 ]
    //            [   0    ]
    //            [   0    ]
    //            [   0    ]
    Eigen::Matrix<double, 6, 1> b;
    b << bp, bn;

    // https://libeigen.gitlab.io/eigen/docs-3.4/classEigen_1_1JacobiSVD.html
    // Compute the SVD of C (6x3).
    // SVD decomposition consists in decomposing any n-by-p matrix A as a product A = U Σ V^* where U is a n-by-n
    // unitary, V is a p-by-p unitary, and Σ is a n-by-p real positive matrix which is zero outside of its main
    // diagonal. The diagonal entries of Σ are known as the singular values of A and the columns of U and V are known as
    // the left and right singular vectors of A respectively.
    // Singular values are always sorted in decreasing order.
    // This JacobiSVD decomposition computes only the singular values by default. If you want U or V, you need to ask
    // for them explicitly.
    // You can ask for only thin U or V to be computed, meaning the following.
    // In case of a rectangular n-by-p matrix, letting m be the smaller value among n and p, there are only m singular
    // vectors; the remaining columns of U and V do not correspond to actual singular vectors.
    // Asking for thin U or V means asking for only their m first columns to be formed.
    // So U is then a n-by-m matrix, and V is then a p-by-m matrix.
    // Notice that thin U and V are all you need for (least squares) solving.

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(C, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Singular values vector, sorted in decreasing order: s_0 ≥ s_1 ≥ s_2 ≥ 0
    const auto& s_values{svd.singularValues()};

    const auto& U{svd.matrixU()};  // 6x3 left sing. vectors, since ComputeThinU was used, measurement space
    const auto& V{svd.matrixV()};  // 3x3 right sing. vectors: xi-space directions
    // const auto Uthin{U.leftCols(V.cols())};  // 6x3 thin U

    constexpr auto rel_thres{1e-6};  // relative singular value truncation threshold to drop weak directions
    constexpr auto lambda{1e-4};     // Tikhonov damping to avoid exploding coefficients on near-degenerate directions

    // Build the damped & truncated inverse of Σ.
    // - Drop directions associated to tiny s_i.
    // - Damp the kept ones so tiny inconsistencies don't explode the solution.
    const double max_s_value{s_values.maxCoeff()};

    // Σ+ = diag(1/s_value_i​)
    //
    //                | 1/s_value_i​)  if sin_value_i​  > (rel_thres * smax)
    // Σ_trunc+(i) = <
    //                | 0               otherwise
    //
    // g_i = s_value_i​ / (s_value_i​^2 + lambda) if (s_value_i​  > (rel_thres * smax)), otherwise 0
    // Σ_reg^{-1} = diag(g_i)
    // diag of Σ_reg^{-1}
    Eigen::Vector3d regularized_inv_s_values{Eigen::Vector3d::Zero()};
    double min_s_value_kept{std::numeric_limits<double>::infinity()};
    auto rank_kept{0};

    for(int i{0}; i < 3; ++i)
    {
      const auto s_value{s_values[i]};
      // If s_value > (rel_thres * smax) -> keep informative directions, with Tikhonov damping,
      // else drop near-null directions to prevent noise blow-up.
      const auto keep{s_value > (rel_thres * max_s_value)};

      if(keep)
      {
        ++rank_kept;
        regularized_inv_s_values[i] = s_value / ((s_value * s_value) + lambda);
        min_s_value_kept            = std::min(min_s_value_kept, s_value);
      }
    }

    // Compute the minimum-norm damped LS solution: twist_vector = V Σ_reg^{-1} U^T b.
    const Eigen::Vector3d twist_vector = V * regularized_inv_s_values.asDiagonal() * U.transpose() * b;

    // Save the result as a Twist message.
    Twist twist;
    twist.vx = twist_vector(0);
    twist.vy = twist_vector(1);
    twist.wz = twist_vector(2);

    // These components are unused/undefined in a ground vehicle.
    twist.vz = std::numeric_limits<double>::quiet_NaN();
    twist.wx = std::numeric_limits<double>::quiet_NaN();
    twist.wy = std::numeric_limits<double>::quiet_NaN();

    // Check health of the solution.
    // Not
    // compute_health_dk_solution(Cp, Cn, bp, max_s_value, min_s_value_kept, static_cast<double>(rank_kept),
    // twist_vector);

    return twist;
  }

  void ThreeSwerveKinematicsSolver::compute_health_dk_solution(const Eigen::Matrix3d& Cp,
                                                               const Eigen::Matrix3d& Cn,
                                                               const Eigen::Vector3d& bp,
                                                               double max_s_value,
                                                               double min_s_value_kept,
                                                               double rank_kept,
                                                               const Eigen::Vector3d& twist_vector)
  {
    // Physically meaningful small velocity to avoid spurious large relative errors.
    // Below this, the robot is effectively stopped.
    constexpr auto v_quasi_static{1e-3};  // m/s

    const auto kappa_kept{rank_kept > 0 ? max_s_value / min_s_value_kept : std::numeric_limits<double>::infinity()};

    // Compute estimates, next residuals (how well we match each block)
    const auto bp_estimate{Cp * twist_vector};
    const auto bn_estimate{Cn * twist_vector};

    // Residuals
    const auto res_p{bp_estimate - bp};  // residual rolling fit
    const auto& res_n{bn_estimate};      // residual lateral no-slip, res_n = bn_estimate - 0 = bn_estimate

    // This is the denominator used to compute relative residuals when the robot is not stopped (i.e., when the robot is
    // moving at a speed larger than v_quasi_static, therefore we can compute relative errors safely).
    const double bp_max{std::max(bp.norm(), bp_estimate.norm())};

    const auto quasi_static{bp_max < v_quasi_static};

    // Two numbers summarizing how large the residuals are.
    double res_p_metric{0.0};
    double res_n_metric{0.0};

    if(quasi_static)
    {
      // Robot is quasi-static, standstill, use absolute residuals, since relative metrics would blow up.
      res_p_metric = res_p.norm();  // m/s
      res_n_metric = res_n.norm();  // m/s
    }
    else
    {
      // Robot is moving: use relative residuals, normalized by motion scale.
      res_p_metric = res_p.norm() / bp_max;  // dimensionless
      res_n_metric = res_n.norm() / bp_max;  // dimensionless
    }

    constexpr auto kappa_warn{50.0};
    constexpr auto kappa_crit{150.0};
    constexpr auto res_warn{0.1};
    constexpr auto res_crit{0.2};

    if(rank_kept < 2 || kappa_kept > kappa_crit || res_p_metric > res_crit || res_n_metric > res_crit)
    {
      std::cout << "[CRITICAL] Direct kinematics solution is unhealthy" << '\n';
      std::cout << "rank_kept: " << rank_kept << '\n';
      std::cout << "kappa_kept: " << kappa_kept << '\n';
      std::cout << "res_p_metric: " << res_p_metric << '\n';
      std::cout << "res_n_metric: " << res_n_metric << '\n';
    }
    else if(kappa_kept > kappa_warn || res_p_metric > res_warn || res_n_metric > res_warn)
    {
      std::cout << "[WARNING] Direct kinematics solution is degraded" << '\n';
      std::cout << "rank_kept: " << rank_kept << '\n';
      std::cout << "kappa_kept: " << kappa_kept << '\n';
      std::cout << "res_p_metric: " << res_p_metric << '\n';
      std::cout << "res_n_metric: " << res_n_metric << '\n';
    }
  }

  //////////////////////////////////////////////////////////////////////////////

  std::array<SteerableWheelCommand, 3> ThreeSwerveKinematicsSolver::solve_inverse(
    const std::array<std::reference_wrapper<const SteerableWheel>, 3>& st_wheels,
    const std::array<std::reference_wrapper<const SteerableWheelState>, 3>& st_wheel_states,
    const Twist& twist,
    const std::array<SteerableWheelCommand, 3>& prev_st_wheel_commands_)
  {
    // Commands for the three steerable wheels to return.
    std::array<SteerableWheelCommand, 3> st_wheel_commands;

    // Process each wheel independently using the inverse kinematics algorithm.
    // 0. 𝐯 = [twist.vx, twist.vy, 0]^T, 𝛚 = [0, 0, twist.wz]
    // 1. Calculate linear velocity of wheel i at the contact point: 𝐯_i = 𝐯 + 𝛚 × 𝐫_i = [vx - wz*y_i, vy + wz*x_i]
    //    𝐫_i is the vector from the robot's frame's origin to the wheel i.
    // 2. Compute steering direction for the wheel i: θ_i = atan2(vy_i, vx_i)
    // 3. Compute wheel angular velocity with rolling speed and radius: 𝛾_i = ||v_i||/wheel_radius

    for(size_t i{0}; i < st_wheels.size(); ++i)
    {
      // CRITICAL: The caller of this function must ensure the st_wheel_state at index i is associated to the st_wheel
      // at the same index.
      const auto& st_wheel              = st_wheels[i].get();
      const auto& st_wheel_state        = st_wheel_states[i].get();
      auto& st_wheel_command            = st_wheel_commands[i];  // Reference to store the computed command for wheel i
      const auto& prev_st_wheel_command = prev_st_wheel_commands_[i];

      // There are two candidate commands, 'cmd_a' and 'cmd_b'.
      // Both commands are computed, then a selection process is applied to choose the 'best one', considering
      // the 'best one' as the command that respects the steering joint limits and minimizes steering motion
      // with respect to the current orientation of the wheel (st_wheel_state).
      SteerableWheelCommand st_wheel_command_a;
      st_wheel_command_a.wheel_name                         = st_wheel.name;
      st_wheel_command_a.rotation_joint_command.joint_name  = st_wheel.rotation_joint.name;
      st_wheel_command_a.steerable_joint_command.joint_name = st_wheel.steerable_joint.name;

      // Check if the robot is effectively stopped (null twist).
      constexpr auto epsilon{0.0001};  // m/s, small enough value to consider a component as null.

      if(std::abs(twist.vx) < epsilon && std::abs(twist.vy) < epsilon && std::abs(twist.wz) < epsilon)
      {
        // Copy the st_wheel_command_a into the st_wheel_commands, so the names (for the wheel and joints) are set
        // properly.
        st_wheel_command = st_wheel_command_a;
        // Null twist means no rotation, keep previous steering angle.
        st_wheel_command.rotation_joint_command.value  = 0.0;
        st_wheel_command.steerable_joint_command.value = prev_st_wheel_command.steerable_joint_command.value;

        // Since the robot is stopped, skip the rest of the computations for this wheel, i.e., no need to
        // compute command 'b' and do the selection process.
        continue;
      }

      // Step 1: Compute the linear velocity of wheel i at the contact point.
      // General equation: 𝐯_i = 𝐯 + 𝛚 × 𝐫_i = [vx - wz*y_i, vy + wz*x_i].
      // Note: the operation a × b can be computed using a skew-symmetric matrix of the form:
      // [a]x = [   0  -az   ay ]
      //        [  az    0  -ax ]
      //        [ -ay   ax    0 ]
      // a x b = [a]x · b = [   0  -az   ay ] [bx] = [-az*by + ay*bz]
      //                    [  az    0  -ax ] [by]   [ az*bx - ax*bz]
      //                    [ -ay   ax    0 ] [bz]   [-ay*bx + ax*by]
      // w x r = [w]x · b = [   0  -wz   0 ] [x] = [-wz*y]
      //                    [  wz    0   0 ] [y]   [ wz*x]
      //                    [   0    0   0 ] [z]   [  0  ]
      const Eigen::Vector2d v{twist.vx - twist.wz * st_wheel.steerable_joint.origin.y,
                              twist.vy + twist.wz * st_wheel.steerable_joint.origin.x};

      // Step 2: Compute desired steering angle from velocity direction.
      // θ_i = atan2(vy_i, vx_i) (direction wheel should point to roll without slipping)
      // Use raw atan2 result (no normalization) so it respects steering limits that may exceed [-pi, pi].
      st_wheel_command_a.steerable_joint_command.value = std::atan2(v.y(), v.x());

      // Step 3:
      // Compute wheel angular velocity with rolling speed and radius: 𝛾_i = ||v_i||/wheel_radius
      st_wheel_command_a.rotation_joint_command.value = (st_wheel.radius > 0.0) ? (v.norm() / st_wheel.radius) : 0.0;

      // DIRECTIONAL AMBIGUITY RESOLUTION:
      //
      // At this point we have calculated the "natural" steering direction (θ_i_a) and
      // rolling speed (𝛾_i_a) needed to achieve the desired velocity vector 𝐯_i.
      //
      // However, there's a fundamental ambiguity in swerve drive kinematics:
      // The SAME velocity vector 𝐯_i can be achieved by TWO different wheel configurations:
      //
      // Configuration A (Computed):
      //   - Wheel points in direction 'θ_i_a'
      //   - Wheel rolls forward with angular velocity '𝛾_i_a'
      //
      // Configuration B (Flipped):
      //   - Wheel points in opposite direction 'θ_i_b = θ_i_a + pi'
      //   - Wheel rolls backward with angular velocity '𝛾_i_b = -𝛾_i_a'
      //
      // Both configurations produce identical contact point velocity 𝐯_i!
      //
      // WHY DO WE NEED THE SW_STATE?
      //
      // Without continuity information, the solver might constantly switch between equivalent configurations, causing:
      // - Sudden 180° steering jumps between control cycles.
      // - Excessive wear on steering actuators.
      // - Jerky robot motion and instability.
      // - Wasted energy from unnecessary large steering motions.
      //
      // SELECTION STRATEGY:
      //
      // We must choose between these configurations considering:
      // 1. Mechanical limits: Does the steering angle fit within joint limits?
      // 2. Continuity: Which option minimizes steering angle change from the current state?
      // 3. Saturation handling: What to do if neither option fits within limits?
      //
      // FLIP-AND-REVERSE STRATEGY:
      //
      // This strategy evaluates both candidates and selects the optimal one:
      // - FLIP: Compute 'θ_i_b' (θ_i_a + pi).
      // - REVERSE: Compute '𝛾_i_b' (-𝛾_i_a).
      // - CHOOSE: Pick candidate that respects limits AND minimizes steering motion relative to st_wheel_states.
      // - FALLBACK: If both violate limits, saturate the best option to nearest boundary.
      //
      // This approach is essential for:
      // - Avoiding unnecessary 180° steering rotations that waste time and energy.
      // - Respecting physical constraints of steering actuators.
      // - Maintaining smooth robot motion without steering discontinuities.
      // - Preventing "thrashing" between equivalent configurations.

      // Select best candidate considering limits and continuity
      SteerableWheelCommand st_wheel_command_b;
      st_wheel_command_b.wheel_name                         = st_wheel.name;
      st_wheel_command_b.rotation_joint_command.joint_name  = st_wheel.rotation_joint.name;
      st_wheel_command_b.steerable_joint_command.joint_name = st_wheel.steerable_joint.name;

      st_wheel_command_b.steerable_joint_command.value = st_wheel_command_a.steerable_joint_command.value + M_PI;
      st_wheel_command_b.rotation_joint_command.value  = -st_wheel_command_a.rotation_joint_command.value;

      st_wheel_command = select_best_sw_state(st_wheel, st_wheel_state, st_wheel_command_a, st_wheel_command_b);
    }

    return st_wheel_commands;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool ThreeSwerveKinematicsSolver::is_within_steering_limits(double angle, const Limits& limits)
  {
    // Since we are comparing floating point numbers, we cannot rely on exact equality (==)
    // The way to 'achive the equality with the limits of the interval' is to use a small tolerance value
    // to extend the interval just a little bit, so now by using > and < we can include the equality case as well.
    constexpr double tolerance{1e-9};

    return (angle > (limits.lower - tolerance)) && (angle < (limits.upper + tolerance));
  }

  //////////////////////////////////////////////////////////////////////////////

  SteerableWheelCommand ThreeSwerveKinematicsSolver::select_best_sw_state(
    const SteerableWheel& st_wheel,
    const SteerableWheelState& st_wheel_state,
    const SteerableWheelCommand& st_wheel_command_a,
    const SteerableWheelCommand& st_wheel_command_b)
  {
    /*
     * SELECTION LOGIC FOR STEERABLE WHEEL STATES
     * ==========================================
     *
     * Context: for any desired contact velocity there are two equivalent steer/roll options:
     * - A) θ_a, γ_a  (natural orientation, forward roll)
     * - B) θ_b = θ_a + pi, γ_b = -γ_a  (flipped orientation, backward roll)
     * Both produce the same contact velocity; we choose the best one under limits and continuity.
     *
     * When both A and B are inside [lower_limit, upper_limit]:
     * - Compute short-path deltas to A and B with angles::shortest_angular_distance.
     * - Compute remaining headroom: dist_upper = upper_limit - theta; dist_lower = theta - lower_limit.
     * - A crosses a limit if:
     *     delta_a > 0 and delta_a > dist_upper   (would overshoot upper), or
     *     delta_a < 0 and -delta_a > dist_lower  (would overshoot lower).
     *   Same test for B with delta_b.
     * - Decision:
     *     * If exactly one path crosses a limit, pick the candidate whose path stays inside.
     *     * If neither crosses (or both would), pick the candidate with smaller |delta| (wrap-aware, avoids 180° flip).
     *
     * When only one candidate is inside limits:
     * - If only A is inside -> return A.
     * - If only B is inside -> return B.
     *
     * Fallback when both are outside:
     * - Pick the one with smaller angular displacement (wrap-aware), clamp its steering angle to the nearest limit,
     *   and return that saturated command (best feasible option).
     */

    // Extract steering angles from candidate commands.
    const auto theta_a{st_wheel_command_a.steerable_joint_command.value};
    const auto theta_b{st_wheel_command_b.steerable_joint_command.value};

    // Determine if we have valid st_wheel_state.
    // To be honest, if 'st_wheel_state.steerable_joint_state.angle' is nan, something is seriously wrong upstream, and
    // we should not be here.
    // However, we handle this gracefully by assuming zero angle as current state, for the time being, but we have
    // to consider checking upstream if 'st_wheel_state.steerable_joint_state.angle' is nan and take action.
    const auto theta{
      !std::isnan(st_wheel_state.steerable_joint_state.angle) ? st_wheel_state.steerable_joint_state.angle : 0.0};

    // PHASE 1: Try to find a candidate that satisfies limits.
    const auto theta_a_within_limits{is_within_steering_limits(theta_a, st_wheel.steerable_joint.limits)};
    const auto theta_b_within_limits{is_within_steering_limits(theta_b, st_wheel.steerable_joint.limits)};

    // Priority: limits compliance, then avoid crossing the tope, then minimize angular displacement.
    if(theta_a_within_limits && theta_b_within_limits)
    {
      // Signed shortest distances from the current state to each candidate (wrap-aware).
      const auto delta_a{angles::shortest_angular_distance(theta, theta_a)};
      const auto delta_b{angles::shortest_angular_distance(theta, theta_b)};
      const auto angular_distance_a{std::fabs(delta_a)};
      const auto angular_distance_b{std::fabs(delta_b)};

      // Check if the short path to each candidate would cross a limit.
      const double dist_upper{st_wheel.steerable_joint.limits.upper - theta};
      const double dist_lower{theta - st_wheel.steerable_joint.limits.lower};
      // Example: theta near upper_limit, delta_a positive and larger than dist_upper => would step past upper_limit.
      // Symmetric for the lower limit when delta is negative.
      const bool a_crosses_limit{(delta_a > 0.0 && delta_a > dist_upper) || (delta_a < 0.0 && -delta_a > dist_lower)};
      const bool b_crosses_limit{(delta_b > 0.0 && delta_b > dist_upper) || (delta_b < 0.0 && -delta_b > dist_lower)};

      // If exactly one path would cross a limit, pick the other candidate.
      // Using xor-like logic (!=) keeps it compact and avoids duplicating the return:
      //   - (true != false) -> true: only A crosses, pick B.
      //   - (false != true) -> true: only B crosses, pick A.
      //   - (false != false) -> false: none crosses, fall through to distance test.
      //   - (true != true)   -> false: both cross, fall through and choose the shorter move anyway
      //     (later clamping still enforces limits).
      if(a_crosses_limit != b_crosses_limit)
      {
        return a_crosses_limit ? st_wheel_command_b : st_wheel_command_a;
      }

      return (angular_distance_a <= angular_distance_b) ? st_wheel_command_a : st_wheel_command_b;
    }
    else if(theta_a_within_limits)
    {
      // Only candidate A satisfies limits
      return st_wheel_command_a;
    }
    else if(theta_b_within_limits)
    {
      // Only candidate B satisfies limits
      return st_wheel_command_b;
    }
    else
    {
      // Neither candidate satisfies limits; choose best option.
      // Signed shortest distances from the current state to each candidate (wrap-aware).
      const auto delta_a{angles::shortest_angular_distance(theta, theta_a)};
      const auto delta_b{angles::shortest_angular_distance(theta, theta_b)};
      const auto angular_distance_a{std::fabs(delta_a)};
      const auto angular_distance_b{std::fabs(delta_b)};

      // Choose candidate with minimum angular displacement from current state
      SteerableWheelCommand chosen_sw_command{(angular_distance_a <= angular_distance_b) ? st_wheel_command_a :
                                                                                           st_wheel_command_b};

      // Saturate steering angle to nearest valid limit
      chosen_sw_command.steerable_joint_command.value = std::clamp(chosen_sw_command.steerable_joint_command.value,
                                                                   st_wheel.steerable_joint.limits.lower,
                                                                   st_wheel.steerable_joint.limits.upper);

      return chosen_sw_command;
    }
  }

}  // namespace ground_vehicle_kinematics
