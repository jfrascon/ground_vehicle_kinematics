#include "ground_vehicle_kinematics/solvers/four_swerve_kinematics.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <unordered_set>

namespace ground_vehicle_kinematics
{
  //////////////////////////////////////////////////////////////////////////////
  // FourSwerveKinematicsSolver
  //////////////////////////////////////////////////////////////////////////////

  FourSwerveKinematicsSolver::FourSwerveKinematicsSolver(const FourSwerveKinematicsSolverConfig& solver_cfg):
    wheel_descriptors_{build_wheel_descriptors(solver_cfg)},
    A_{build_A()},
    direct_k_solver_{Eigen::MatrixXd{A_}, solver_cfg.svd_solver_cfg()}
  {
    // Validate relationships between the configured wheels.
    validate();
  }

  Eigen::Matrix<double, 8, 3> FourSwerveKinematicsSolver::build_A() const
  {
    Eigen::Matrix<double, 8, 3> A{Eigen::Matrix<double, 8, 3>::Zero()};

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

  Eigen::Matrix<double, 8, 1> FourSwerveKinematicsSolver::build_b() const
  {
    Eigen::Matrix<double, 8, 1> b{Eigen::Matrix<double, 8, 1>::Zero()};

    for(std::size_t i{0}; i < wheel_descriptors_.size(); ++i)
    {
      const auto& wheel{wheel_descriptors_[i].wheel()};
      const auto& wheel_state{wheel_descriptors_[i].wheel_state()};
      const double theta{wheel_state.steering_joint_state().angle()};
      const double ang_vel{wheel_state.rotation_joint_state().ang_vel()};

      if(!std::isfinite(theta) || !std::isfinite(ang_vel))
      {
        throw std::invalid_argument("FourSwerveKinematicsSolver requires finite steering angles and wheel speeds.");
      }

      const double wheel_speed{wheel.radius() * ang_vel};
      b((2 * i), 0)     = wheel_speed * std::cos(theta);
      b((2 * i) + 1, 0) = wheel_speed * std::sin(theta);
    }

    return b;
  }

  void FourSwerveKinematicsSolver::build_wheel_commands(const Twist& twist_cmd)
  {
    const Eigen::Vector3d chassis_twist{twist_cmd.vx(), twist_cmd.vy(), twist_cmd.wz()};
    const Eigen::Matrix<double, 8, 1> b{A_ * chassis_twist};

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

  SteerableWheel FourSwerveKinematicsSolver::build_wheel(const SteerableWheelConfig& wheel_cfg)
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

  std::array<SteerableWheelDescriptor, 4> FourSwerveKinematicsSolver::build_wheel_descriptors(
    const FourSwerveKinematicsSolverConfig& solver_cfg)
  {
    const auto& wheel_cfgs{solver_cfg.wheel_cfgs()};

    return {SteerableWheelDescriptor{build_wheel(wheel_cfgs[0])},
            SteerableWheelDescriptor{build_wheel(wheel_cfgs[1])},
            SteerableWheelDescriptor{build_wheel(wheel_cfgs[2])},
            SteerableWheelDescriptor{build_wheel(wheel_cfgs[3])}};
  }

  Twist FourSwerveKinematicsSolver::get_twist(std::array<SteerableWheelState, 4> wheel_states)
  {
    set_wheel_states(wheel_states);

    // Return the solved twist based on the updated wheel descriptors.
    return solve_direct();
  }

  std::array<std::reference_wrapper<const SteerableWheelCommand>, 4> FourSwerveKinematicsSolver::get_wheel_commands(
    const Twist& twist_cmd)
  {
    build_wheel_commands(twist_cmd);

    return {std::cref(wheel_descriptors_[0].wheel_command()),
            std::cref(wheel_descriptors_[1].wheel_command()),
            std::cref(wheel_descriptors_[2].wheel_command()),
            std::cref(wheel_descriptors_[3].wheel_command())};
  }

  const SteerableWheel& FourSwerveKinematicsSolver::wheel(const std::size_t wheel_index) const
  {
    if(wheel_index >= wheel_descriptors_.size())
    {
      throw std::out_of_range("FourSwerveKinematicsSolver wheel index must be in range [0, 3].");
    }

    return wheel_descriptors_[wheel_index].wheel();
  }

  void FourSwerveKinematicsSolver::set_wheel_states(const std::array<SteerableWheelState, 4>& wheel_states)
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

  Twist FourSwerveKinematicsSolver::solve_direct() const
  {
    const Eigen::Vector3d twist_vector{direct_k_solver_.solve(build_b())};

    return Twist{twist_vector(0),
                 twist_vector(1),
                 0.0,
                 0.0,
                 0.0,
                 twist_vector(2),
                 wheel_descriptors_[0].wheel_state().rotation_joint_state().timestamp_ns()};
  }

  void FourSwerveKinematicsSolver::validate() const
  {
    constexpr double geometry_tolerance{1.0e-9};

    const auto nearly_equal{[](const double lhs, const double rhs, const double tolerance) {
      return std::abs(lhs - rhs) <= tolerance;
    }};

    std::unordered_set<std::string> names;

    const auto& w0{wheel_descriptors_[0].wheel()};
    const auto& w1{wheel_descriptors_[1].wheel()};
    const auto& w2{wheel_descriptors_[2].wheel()};
    const auto& w3{wheel_descriptors_[3].wheel()};

    for(const auto& wheel: {w0, w1, w2, w3})
    {
      names.insert(wheel.name());
      names.insert(wheel.rotation_joint().name());
      names.insert(wheel.steering_joint().name());
    }

    if(names.size() != 12U)
    {
      throw std::invalid_argument(
        "FourSwerveKinematicsSolver requires all wheel and joint names to be globally unique.");
    }

    if(!nearly_equal(w0.radius(), w1.radius(), geometry_tolerance) ||
       !nearly_equal(w0.radius(), w2.radius(), geometry_tolerance) ||
       !nearly_equal(w0.radius(), w3.radius(), geometry_tolerance))
    {
      throw std::invalid_argument("FourSwerveKinematicsSolver requires equal wheel radii.");
    }

    if(!nearly_equal(w0.dist(), w1.dist(), geometry_tolerance) ||
       !nearly_equal(w0.dist(), w2.dist(), geometry_tolerance) ||
       !nearly_equal(w0.dist(), w3.dist(), geometry_tolerance))
    {
      throw std::invalid_argument("FourSwerveKinematicsSolver requires equal wheel distances to the base origin.");
    }

    if(!nearly_equal(w0.beta(), (M_PI / 2.0) - w0.alpha(), geometry_tolerance) ||
       !nearly_equal(w1.beta(), (M_PI / 2.0) - w1.alpha(), geometry_tolerance) ||
       !nearly_equal(w2.beta(), (M_PI / 2.0) - w2.alpha(), geometry_tolerance) ||
       !nearly_equal(w3.beta(), (M_PI / 2.0) - w3.alpha(), geometry_tolerance))
    {
      throw std::invalid_argument("FourSwerveKinematicsSolver requires beta_i = pi/2 - alpha_i.");
    }

    const auto& pos0{w0.pos2d()};
    const auto& pos1{w1.pos2d()};
    const auto& pos2{w2.pos2d()};
    const auto& pos3{w3.pos2d()};

    if(!(pos0.x() > geometry_tolerance && pos1.x() > geometry_tolerance && pos2.x() < -geometry_tolerance &&
         pos3.x() < -geometry_tolerance))
    {
      throw std::invalid_argument(
        "FourSwerveKinematicsSolver requires wheels 0 and 1 to be in front, and wheels 2 and 3 to be at the rear.");
    }

    if(!(pos0.y() > geometry_tolerance && pos2.y() > geometry_tolerance && pos1.y() < -geometry_tolerance &&
         pos3.y() < -geometry_tolerance))
    {
      throw std::invalid_argument(
        "FourSwerveKinematicsSolver requires wheels 0 and 2 on the left, and wheels 1 and 3 on the right.");
    }

    if(!nearly_equal(pos0.x(), pos1.x(), geometry_tolerance) || !nearly_equal(pos2.x(), pos3.x(), geometry_tolerance) ||
       !nearly_equal(pos2.x(), -pos0.x(), geometry_tolerance))
    {
      throw std::invalid_argument(
        "FourSwerveKinematicsSolver requires front and rear wheel x coordinates to form a symmetric rectangle.");
    }

    if(!nearly_equal(pos0.y(), -pos1.y(), geometry_tolerance) ||
       !nearly_equal(pos2.y(), -pos3.y(), geometry_tolerance) || !nearly_equal(pos2.y(), pos0.y(), geometry_tolerance))
    {
      throw std::invalid_argument(
        "FourSwerveKinematicsSolver requires left and right wheel y coordinates to form a symmetric rectangle.");
    }
  }
}  // namespace ground_vehicle_kinematics
