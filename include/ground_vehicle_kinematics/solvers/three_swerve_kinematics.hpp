#pragma once

/**
 * @file three_swerve_kinematics.hpp
 * @brief Three-swerve direct and inverse kinematics solver.
 */

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <functional>

#include "ground_vehicle_kinematics/solvers/svd_least_squares_solver.hpp"
#include "ground_vehicle_kinematics/types.hpp"

namespace ground_vehicle_kinematics
{
  /**
   * @brief Configuration used to build the three-swerve kinematics solver.
   */
  class ThreeSwerveKinematicsSolverConfig
  {
    public:
      /**
       * @brief Build one three-swerve solver config.
       * @param wheel_cfgs Configs of the three steerable wheels in documented wheel order.
       * @param svd_solver_cfg Numerical config of the SVD least-squares solver.
       */
      ThreeSwerveKinematicsSolverConfig(const std::array<SteerableWheelConfig, 3>& wheel_cfgs,
                                        const SvdLeastSquaresSolverConfig& svd_solver_cfg):
        wheel_cfgs_{wheel_cfgs},
        svd_solver_cfg_{svd_solver_cfg}
      {}

      /** @return Numerical config of the SVD least-squares solver. */
      const SvdLeastSquaresSolverConfig& svd_solver_cfg() const
      {
        return svd_solver_cfg_;
      }

      /** @return Configs of the three steerable wheels in documented wheel order. */
      const std::array<SteerableWheelConfig, 3>& wheel_cfgs() const
      {
        return wheel_cfgs_;
      }

    private:
      std::array<SteerableWheelConfig, 3> wheel_cfgs_;
      SvdLeastSquaresSolverConfig svd_solver_cfg_;
  };

  /**
   * @brief Solver for the three-swerve model.
   */
  class ThreeSwerveKinematicsSolver
  {
    public:
      /**
       * @brief Build the solver from its configuration.
       * @param solver_cfg Solver configuration.
       * @throws std::invalid_argument If the configuration is inconsistent.
       */
      explicit ThreeSwerveKinematicsSolver(const ThreeSwerveKinematicsSolverConfig& solver_cfg);

      /**
       * @brief Get diagnostics of the direct-kinematics solver.
       * @return Const reference to the current direct-kinematics solver diagnostics.
       */
      const SvdLeastSquaresDiagnostics& diagnostics() const noexcept
      {
        return direct_k_solver_.diagnostics();
      }

      /**
       * @brief Store the three wheel states and return the chassis twist.
       * @param wheel_states Wheel states in documented wheel order.
       * @return Chassis twist.
       * @throws std::invalid_argument If wheel states do not match the configured wheels.
       */
      Twist get_twist(std::array<SteerableWheelState, 3> wheel_states);

      /**
       * @brief Build the three wheel commands from a chassis twist, store them, and return them.
       * @param twist_cmd Desired chassis twist command.
       * @return Wheel commands in documented wheel order.
       */
      std::array<std::reference_wrapper<const SteerableWheelCommand>, 3> get_wheel_commands(const Twist& twist_cmd);

      /**
       * @brief Get one configured wheel by index.
       * @param wheel_index Wheel index in documented wheel order.
       * @return Const reference to the requested wheel.
       * @throws std::out_of_range If @p wheel_index is outside [0, 2].
       */
      const SteerableWheel& wheel(std::size_t wheel_index) const;

    private:
      /**
       * @brief Number of wheels in the three-swerve model.
       */
      static constexpr std::size_t wheel_count_{3};

      /**
       * @brief Build the constant matrix A from the stored wheel positions.
       * @return Direct-kinematics matrix A.
       */
      Eigen::Matrix<double, 6, 3> build_A() const;

      /**
       * @brief Build the current right-hand side vector b from the stored wheel descriptors.
       * @return Right-hand side vector b.
       * @throws std::invalid_argument If direct-kinematics inputs are not finite.
       */
      Eigen::Matrix<double, 6, 1> build_b() const;

      /**
       * @brief Build the three wheel commands associated with one chassis twist.
       * @param twist_cmd Desired chassis twist command.
       */
      void build_wheel_commands(const Twist& twist_cmd);

      /**
       * @brief Build one steerable wheel from one wheel config.
       * @param wheel_cfg Wheel config.
       * @return One steerable wheel.
       */
      static SteerableWheel build_wheel(const SteerableWheelConfig& wheel_cfg);

      /**
       * @brief Build the three wheel descriptors from the solver config.
       * @param solver_cfg Solver configuration.
       * @return Wheel descriptors in documented wheel order.
       */
      static std::array<SteerableWheelDescriptor, 3> build_wheel_descriptors(
        const ThreeSwerveKinematicsSolverConfig& solver_cfg);

      /**
       * @brief Update the three wheel descriptors with new wheel states.
       * @param wheel_states Wheel states in documented wheel order.
       * @throws std::invalid_argument If wheel states do not match the stored wheel descriptors.
       */
      void set_wheel_states(const std::array<SteerableWheelState, 3>& wheel_states);

      /**
       * @brief Solve direct kinematics from the currently stored wheel descriptors.
       * @return Chassis twist.
       */
      Twist solve_direct() const;

      /**
       * @brief Validate the three configured wheels before using them.
       * @throws std::invalid_argument If wheel names, joint names or geometry are inconsistent.
       */
      void validate() const;

      /** @brief Stored wheel descriptors in documented wheel order. */
      std::array<SteerableWheelDescriptor, 3> wheel_descriptors_;

      /**
       * @brief Constant matrix A of the platform model.
       */
      Eigen::Matrix<double, 6, 3> A_;

      /**
       * @brief Least-squares solver used by direct kinematics.
       */
      SvdLeastSquaresSolver direct_k_solver_;
  };

}  // namespace ground_vehicle_kinematics
