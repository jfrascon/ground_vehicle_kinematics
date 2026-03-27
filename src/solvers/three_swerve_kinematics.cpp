#include "ground_vehicle_kinematics/solvers/three_swerve_kinematics.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <unordered_set>

namespace ground_vehicle_kinematics
{
  //////////////////////////////////////////////////////////////////////////////
  // ThreeSwerveKinematicsSolver
  //////////////////////////////////////////////////////////////////////////////

  ThreeSwerveKinematicsSolver::ThreeSwerveKinematicsSolver(const ThreeSwerveKinematicsSolverConfig& solver_cfg):
    wheel_descriptors_{build_wheel_descriptors(solver_cfg)},
    A_{build_A()},
    direct_k_solver_{Eigen::MatrixXd{A_}, solver_cfg.svd_solver_cfg()}
  {
    // Validate relationships between the configured wheels.
    validate();
  }

  Eigen::Matrix<double, 6, 3> ThreeSwerveKinematicsSolver::build_A() const
  {
    Eigen::Matrix<double, 6, 3> A{Eigen::Matrix<double, 6, 3>::Zero()};

    for(std::size_t i{0}; i < wheel_descriptors_.size(); ++i)
    {
      const auto& pos2d{wheel_descriptors_[i].wheel().pos2d()};

      A((2 * i), 0)     = 1.0;
      A((2 * i), 1)     = 0.0;
      A((2 * i), 2)     = -pos2d.y();
      A((2 * i) + 1, 0) = 0.0;
      A((2 * i) + 1, 1) = 1.0;
      A((2 * i) + 1, 2) = pos2d.x();
    }

    return A;
  }

  Eigen::Matrix<double, 6, 1> ThreeSwerveKinematicsSolver::build_b() const
  {
    Eigen::Matrix<double, 6, 1> b{Eigen::Matrix<double, 6, 1>::Zero()};

    for(std::size_t i{0}; i < wheel_descriptors_.size(); ++i)
    {
      const auto& wheel{wheel_descriptors_[i].wheel()};
      const auto& wheel_state{wheel_descriptors_[i].wheel_state()};
      const double theta{wheel_state.steering_joint_state().angle()};
      const double ang_vel{wheel_state.rotation_joint_state().ang_vel()};

      if(!std::isfinite(theta) || !std::isfinite(ang_vel))
      {
        throw std::invalid_argument("ThreeSwerveKinematicsSolver requires finite steering angles and wheel speeds.");
      }

      const double wheel_speed{wheel.radius() * ang_vel};
      b((2 * i), 0)     = wheel_speed * std::cos(theta);
      b((2 * i) + 1, 0) = wheel_speed * std::sin(theta);
    }

    return b;
  }

  void ThreeSwerveKinematicsSolver::build_wheel_commands(const Twist& twist_cmd)
  {
    const Eigen::Vector3d chassis_twist{twist_cmd.vx(), twist_cmd.vy(), twist_cmd.wz()};
    const Eigen::Matrix<double, 6, 1> b{A_ * chassis_twist};

    for(std::size_t i{0}; i < wheel_descriptors_.size(); ++i)
    {
      const auto& wheel_desc{wheel_descriptors_[i]};
      const auto& wheel{wheel_desc.wheel()};
      const auto& wheel_state{wheel_desc.wheel_state()};

      const double wheel_vel_x{b(2 * i)};
      const double wheel_vel_y{b((2 * i) + 1)};

      const double theta_a{std::atan2(wheel_vel_y, wheel_vel_x)};
      const double wheel_ang_vel_a{std::hypot(wheel_vel_x, wheel_vel_y) / wheel.radius()};

      wheel_descriptors_[i].set_wheel_command(
        SteerableWheelCommand{wheel, wheel_state, theta_a, wheel_ang_vel_a, twist_cmd.timestamp_ns()});
    }
  }

  SteerableWheel ThreeSwerveKinematicsSolver::build_wheel(const SteerableWheelConfig& wheel_cfg)
  {
    return SteerableWheel{wheel_cfg.radius(),
                          wheel_cfg.dist(),
                          wheel_cfg.alpha(),
                          wheel_cfg.beta(),
                          wheel_cfg.wheel_name(),
                          RotationJoint{wheel_cfg.rotation_joint_name(),
                                        Limits{wheel_cfg.rotation_joint_lower(), wheel_cfg.rotation_joint_upper()}},
                          SteeringJoint{wheel_cfg.steering_joint_name(),
                                        Limits{wheel_cfg.steering_joint_lower(), wheel_cfg.steering_joint_upper()}}};
  }

  std::array<SteerableWheelDescriptor, 3> ThreeSwerveKinematicsSolver::build_wheel_descriptors(
    const ThreeSwerveKinematicsSolverConfig& solver_cfg)
  {
    const auto& wheel_cfgs{solver_cfg.wheel_cfgs()};

    return {SteerableWheelDescriptor{build_wheel(wheel_cfgs[0])},
            SteerableWheelDescriptor{build_wheel(wheel_cfgs[1])},
            SteerableWheelDescriptor{build_wheel(wheel_cfgs[2])}};
  }

  Twist ThreeSwerveKinematicsSolver::get_twist(std::array<SteerableWheelState, 3> wheel_states)
  {
    set_wheel_states(wheel_states);

    // Return the solved twist based on the updated wheel descriptors.
    return solve_direct();
  }

  std::array<std::reference_wrapper<const SteerableWheelCommand>, 3> ThreeSwerveKinematicsSolver::get_wheel_commands(
    const Twist& twist_cmd)
  {
    build_wheel_commands(twist_cmd);

    return {std::cref(wheel_descriptors_[0].wheel_command()),
            std::cref(wheel_descriptors_[1].wheel_command()),
            std::cref(wheel_descriptors_[2].wheel_command())};
  }

  const SteerableWheel& ThreeSwerveKinematicsSolver::wheel(const std::size_t wheel_index) const
  {
    if(wheel_index >= wheel_descriptors_.size())
    {
      throw std::out_of_range("ThreeSwerveKinematicsSolver wheel index must be in range [0, 2].");
    }

    return wheel_descriptors_[wheel_index].wheel();
  }

  void ThreeSwerveKinematicsSolver::set_wheel_states(const std::array<SteerableWheelState, 3>& wheel_states)
  {
    for(std::size_t i{0}; i < wheel_states.size(); ++i)
    {
      const auto& wheel = wheel_descriptors_[i].wheel();

      if(!wheel_states[i].matches(wheel))
      {
        throw std::invalid_argument("Wheel state does not match configured wheel at index " + std::to_string(i) + ".");
      }
    }

    for(std::size_t i{0}; i < wheel_states.size(); ++i)
    {
      wheel_descriptors_[i].set_wheel_state(wheel_states[i]);
    }
  }

  Twist ThreeSwerveKinematicsSolver::solve_direct() const
  {
    // wheel_states => twist for the robot.
    const Eigen::Vector3d twist_vector{direct_k_solver_.solve(build_b())};

    // Twist contains a timestamp field.
    // In this case, we set the timestamp to the value of any timestamp of the wheel states, since they should all be
    // equal. We use the timestamp of the wheel 0.
    return Twist{twist_vector(0),
                 twist_vector(1),
                 0.0,
                 0.0,
                 0.0,
                 twist_vector(2),
                 wheel_descriptors_[0].wheel_state().rotation_joint_state().timestamp_ns()};
  }

  void ThreeSwerveKinematicsSolver::validate() const
  {
    // Check that all wheel and joint names are globally unique, that all wheel radii and distances to the base origin
    // are equal, that alpha_1 lies within [0, pi], that alpha_2 lies within [-pi, 0], that alpha_2 = -alpha_1, that
    // alpha_0 = 0 when alpha_1 < pi/2 or alpha_0 = pi otherwise, and that beta_i = pi/2 - alpha_i for all i.

    constexpr double geometry_tolerance{1.0e-9};

    const auto nearly_equal{[](const double lhs, const double rhs, const double tolerance) {
      return std::abs(lhs - rhs) <= tolerance;
    }};

    std::unordered_set<std::string> names;

    const auto& w0{wheel_descriptors_[0].wheel()};
    const auto& w1{wheel_descriptors_[1].wheel()};
    const auto& w2{wheel_descriptors_[2].wheel()};

    for(const auto& wheel: {w0, w1, w2})
    {
      names.insert(wheel.name());
      names.insert(wheel.rotation_joint().name());
      names.insert(wheel.steering_joint().name());
    }

    if(names.size() != 9U)
    {
      throw std::invalid_argument(
        "ThreeSwerveKinematicsSolver requires all wheel and joint names to be globally unique.");
    }

    if(!nearly_equal(w0.radius(), w1.radius(), geometry_tolerance) ||
       !nearly_equal(w0.radius(), w2.radius(), geometry_tolerance))
    {
      throw std::invalid_argument("ThreeSwerveKinematicsSolver requires equal wheel radii.");
    }

    if(!nearly_equal(w0.dist(), w1.dist(), geometry_tolerance) ||
       !nearly_equal(w0.dist(), w2.dist(), geometry_tolerance))
    {
      throw std::invalid_argument("ThreeSwerveKinematicsSolver requires equal wheel distances to the base origin.");
    }

    if(w1.alpha() < 0.0 || w1.alpha() > M_PI)
    {
      throw std::invalid_argument("ThreeSwerveKinematicsSolver requires alpha_1 to lie within [0, pi].");
    }

    if(w2.alpha() < -M_PI || w2.alpha() > 0.0)
    {
      throw std::invalid_argument("ThreeSwerveKinematicsSolver requires alpha_2 to lie within [-pi, 0].");
    }

    if(!nearly_equal(w2.alpha(), -w1.alpha(), geometry_tolerance))
    {
      throw std::invalid_argument("ThreeSwerveKinematicsSolver requires alpha_2 = -alpha_1.");
    }

    const double expected_alpha_0{(w1.alpha() < (M_PI / 2.0)) ? 0.0 : M_PI};

    if(!nearly_equal(w0.alpha(), expected_alpha_0, geometry_tolerance))
    {
      throw std::invalid_argument(
        "ThreeSwerveKinematicsSolver requires alpha_0 = 0 when alpha_1 < pi/2, or alpha_0 = pi otherwise.");
    }

    if(!nearly_equal(w0.beta(), (M_PI / 2.0) - w0.alpha(), geometry_tolerance) ||
       !nearly_equal(w1.beta(), (M_PI / 2.0) - w1.alpha(), geometry_tolerance) ||
       !nearly_equal(w2.beta(), (M_PI / 2.0) - w2.alpha(), geometry_tolerance))
    {
      throw std::invalid_argument("ThreeSwerveKinematicsSolver requires beta_i = pi/2 - alpha_i.");
    }
  }
}  // namespace ground_vehicle_kinematics
