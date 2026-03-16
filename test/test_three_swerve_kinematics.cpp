#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>

#include "ground_vehicle_kinematics/solvers/three_swerve_kinematics.hpp"

namespace ground_vehicle_kinematics
{
  namespace
  {
    ThreeSwerveKinematicsSolverConfig make_three_swerve_solver_config()
    {
      constexpr double wheel_radius{0.10};
      constexpr double dist{0.50};
      constexpr double alpha_0{M_PI};
      constexpr double alpha_1{2.6878070480712677};
      constexpr double alpha_2{-2.6878070480712677};
      constexpr double beta_0{(M_PI / 2.0) - alpha_0};
      constexpr double beta_1{(M_PI / 2.0) - alpha_1};
      constexpr double beta_2{(M_PI / 2.0) - alpha_2};

      return ThreeSwerveKinematicsSolverConfig{
        std::array<SteerableWheelConfig, 3>{SteerableWheelConfig{wheel_radius,
                                                                 dist,
                                                                 alpha_0,
                                                                 beta_0,
                                                                 "wheel_0",
                                                                 "wheel_0_rotation_joint",
                                                                 -100.0,
                                                                 100.0,
                                                                 "wheel_0_steer_joint",
                                                                 -2.0 * M_PI,
                                                                 2.0 * M_PI},
                                            SteerableWheelConfig{wheel_radius,
                                                                 dist,
                                                                 alpha_1,
                                                                 beta_1,
                                                                 "wheel_1",
                                                                 "wheel_1_rotation_joint",
                                                                 -100.0,
                                                                 100.0,
                                                                 "wheel_1_steer_joint",
                                                                 -2.0 * M_PI,
                                                                 2.0 * M_PI},
                                            SteerableWheelConfig{wheel_radius,
                                                                 dist,
                                                                 alpha_2,
                                                                 beta_2,
                                                                 "wheel_2",
                                                                 "wheel_2_rotation_joint",
                                                                 -100.0,
                                                                 100.0,
                                                                 "wheel_2_steer_joint",
                                                                 -2.0 * M_PI,
                                                                 2.0 * M_PI}},
        SvdLeastSquaresSolverConfig{0.0, 0.0}};
    }

    ThreeSwerveKinematicsSolver make_three_swerve_solver()
    {
      return ThreeSwerveKinematicsSolver{make_three_swerve_solver_config()};
    }

    std::array<SteerableWheelState, 3> build_wheel_states_from_twist(
      const ThreeSwerveKinematicsSolverConfig& solver_cfg,
      const Twist& twist,
      const int64_t timestamp_ns = 0)
    {
      const auto& wheel_cfgs{solver_cfg.wheel_cfgs()};
      std::array<SteerableWheelState, 3> wheel_states;

      for(std::size_t i{0}; i < wheel_cfgs.size(); ++i)
      {
        const auto& wheel_cfg{wheel_cfgs[i]};
        const double wheel_pos_x{wheel_cfg.dist() * std::cos(wheel_cfg.alpha())};
        const double wheel_pos_y{wheel_cfg.dist() * std::sin(wheel_cfg.alpha())};
        const double wheel_vel_x{twist.vx() - twist.wz() * wheel_pos_y};
        const double wheel_vel_y{twist.vy() + twist.wz() * wheel_pos_x};

        wheel_states[i] = SteerableWheelState{
          wheel_cfg.wheel_name(),
          RotationJointState{wheel_cfg.rotation_joint_name(),
                             0.0,
                             std::hypot(wheel_vel_x, wheel_vel_y) / wheel_cfg.radius(),
                             timestamp_ns},
          SteerableJointState{wheel_cfg.steerable_joint_name(),
                              std::atan2(wheel_vel_y, wheel_vel_x),
                              0.0,
                              timestamp_ns}};
      }

      return wheel_states;
    }
  }  // namespace

  TEST(ThreeSwerveKinematicsSolverTest, ReturnsWheelCommandsInDocumentedWheelOrder)
  {
    ThreeSwerveKinematicsSolver solver{make_three_swerve_solver()};
    const auto solver_cfg{make_three_swerve_solver_config()};
    const Twist wheel_state_source_twist{0.2, -0.1, 0.0, 0.0, 0.0, 0.15};
    const auto wheel_states{build_wheel_states_from_twist(solver_cfg, wheel_state_source_twist, 123U)};

    (void)solver.get_twist(wheel_states);

    const auto wheel_cmds{solver.get_wheel_commands(Twist{0.4, 0.25, 0.0, 0.0, 0.0, -0.3, 456U})};

    EXPECT_EQ(wheel_cmds[0].get().wheel_name(), "wheel_0");
    EXPECT_EQ(wheel_cmds[1].get().wheel_name(), "wheel_1");
    EXPECT_EQ(wheel_cmds[2].get().wheel_name(), "wheel_2");
  }

  TEST(ThreeSwerveKinematicsSolverTest, ComputesTwistFromRawWheelData)
  {
    ThreeSwerveKinematicsSolver solver{make_three_swerve_solver()};
    const auto solver_cfg{make_three_swerve_solver_config()};
    const Twist expected_twist{0.8, -0.2, 0.0, 0.0, 0.0, 0.35};
    const auto wheel_states{build_wheel_states_from_twist(solver_cfg, expected_twist, 0U)};

    const Twist twist{solver.get_twist(wheel_states)};

    EXPECT_NEAR(twist.vx(), expected_twist.vx(), 1e-9);
    EXPECT_NEAR(twist.vy(), expected_twist.vy(), 1e-9);
    EXPECT_NEAR(twist.wz(), expected_twist.wz(), 1e-9);
  }

  TEST(ThreeSwerveKinematicsSolverTest, RoundTripsJointCommandsAndTwist)
  {
    ThreeSwerveKinematicsSolver solver{make_three_swerve_solver()};
    const auto solver_cfg{make_three_swerve_solver_config()};
    const Twist initial_state_twist{0.2, -0.1, 0.0, 0.0, 0.0, 0.15};
    const Twist commanded_twist{0.4, 0.25, 0.0, 0.0, 0.0, -0.3};
    const auto initial_wheel_states{build_wheel_states_from_twist(solver_cfg, initial_state_twist, 456U)};

    (void)solver.get_twist(initial_wheel_states);

    const auto wheel_cmds{solver.get_wheel_commands(Twist{commanded_twist.vx(),
                                                          commanded_twist.vy(),
                                                          commanded_twist.vz(),
                                                          commanded_twist.wx(),
                                                          commanded_twist.wy(),
                                                          commanded_twist.wz(),
                                                          789U})};

    std::array<SteerableWheelState, 3> wheel_states;

    for(std::size_t i{0}; i < wheel_cmds.size(); ++i)
    {
      const auto& wheel_cfg{solver_cfg.wheel_cfgs()[i]};
      wheel_states[i] = SteerableWheelState{wheel_cfg.wheel_name(),
                                            RotationJointState{wheel_cfg.rotation_joint_name(),
                                                               0.0,
                                                               wheel_cmds[i].get().rotation_joint_command().value(),
                                                               999U},
                                            SteerableJointState{wheel_cfg.steerable_joint_name(),
                                                                wheel_cmds[i].get().steerable_joint_command().value(),
                                                                0.0,
                                                                999U}};
    }

    const Twist twist{solver.get_twist(wheel_states)};

    EXPECT_NEAR(twist.vx(), commanded_twist.vx(), 1e-9);
    EXPECT_NEAR(twist.vy(), commanded_twist.vy(), 1e-9);
    EXPECT_NEAR(twist.wz(), commanded_twist.wz(), 1e-9);
  }

}  // namespace ground_vehicle_kinematics
