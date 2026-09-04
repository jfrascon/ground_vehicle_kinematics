#include "ground_vehicle_kinematics/four_swerve_kinematics_ros.hpp"

#include "ground_vehicle_kinematics/types.hpp"

#include <array>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/parameter.hpp"

namespace ground_vehicle_kinematics
{
  FourSwerveKinematicsSolverRos::FourSwerveKinematicsSolverRos(const rclcpp::NodeOptions& options):
    rclcpp::Node("four_swerve_kinematics_solver", options),
    constructor_logger_(this->get_logger().get_child("constructor")),
    create_solver_config_logger_(this->get_logger().get_child("create_solver_config")),
    twist_cb_logger_(this->get_logger().get_child("twist_cb")),
    joint_state_cb_logger_(this->get_logger().get_child("joint_state_cb"))
  {
    this->declare_parameter<double>("direct_solver.relative_singular_value_threshold", 1e-6);
    this->declare_parameter<double>("direct_solver.tikhonov_lambda", 1e-4);
    this->declare_parameter<double>("initial_wheel_states_reception_period", 10.0);

    // The wheel order is front-left, front-right, rear-left, and rear-right.
    for(std::size_t i{0}; i < wheel_paths_.size(); ++i)
    {
      const std::string& wheel_path{wheel_paths_[i]};
      this->declare_parameter<double>(wheel_path + ".radius");
      this->declare_parameter<double>(wheel_path + ".dist");
      this->declare_parameter<double>(wheel_path + ".alpha");
      this->declare_parameter<double>(wheel_path + ".beta");
      this->declare_parameter<std::string>(wheel_path + ".name");
      this->declare_parameter<std::string>(wheel_path + ".steering_joint.name");
      this->declare_parameter<double>(wheel_path + ".steering_joint.limits.lower");
      this->declare_parameter<double>(wheel_path + ".steering_joint.limits.upper");
      this->declare_parameter<std::string>(wheel_path + ".rotation_joint.name");
      this->declare_parameter<double>(wheel_path + ".rotation_joint.limits.lower");
      this->declare_parameter<double>(wheel_path + ".rotation_joint.limits.upper");
    }

    const auto initial_wheel_states_reception_period{
      this->get_parameter("initial_wheel_states_reception_period").get_value<double>()};

    if(initial_wheel_states_reception_period > 0.0)
    {
      t_start_pub_commands_ = this->now() + rclcpp::Duration::from_seconds(initial_wheel_states_reception_period);
    }
    else
    {
      RCLCPP_WARN(constructor_logger_,
                  "Initial wheel states reception period is non-positive (%.2f). No period used",
                  initial_wheel_states_reception_period);

      t_start_pub_commands_ = this->now();
    }

    solver_ = std::make_unique<FourSwerveKinematicsSolver>(create_solver_config());

    create_subs_pubs();

    RCLCPP_INFO(constructor_logger_, "FourSwerveKinematicsSolverRos node initialized.");
  }

  FourSwerveKinematicsSolverConfig FourSwerveKinematicsSolverRos::create_solver_config() const
  {
    auto create_wheel_config = [this](size_t wheel_index) -> SteerableWheelConfig {
      if(wheel_index >= wheel_paths_.size())
      {
        throw std::out_of_range("FourSwerveKinematicsSolverRos wheel index must be in range [0, 3].");
      }

      const auto& wheel_path{wheel_paths_[wheel_index]};

      const SteerableWheelConfig
        wheel_cfg{this->get_parameter(wheel_path + ".radius").get_value<double>(),
                  this->get_parameter(wheel_path + ".dist").get_value<double>(),
                  this->get_parameter(wheel_path + ".alpha").get_value<double>(),
                  this->get_parameter(wheel_path + ".beta").get_value<double>(),
                  this->get_parameter(wheel_path + ".name").as_string(),
                  this->get_parameter(wheel_path + ".rotation_joint.name").as_string(),
                  this->get_parameter(wheel_path + ".rotation_joint.limits.lower").get_value<double>(),
                  this->get_parameter(wheel_path + ".rotation_joint.limits.upper").get_value<double>(),
                  this->get_parameter(wheel_path + ".steering_joint.name").as_string(),
                  this->get_parameter(wheel_path + ".steering_joint.limits.lower").get_value<double>(),
                  this->get_parameter(wheel_path + ".steering_joint.limits.upper").get_value<double>()};

      RCLCPP_INFO(create_solver_config_logger_,
                  "Processed configuration for steerable wheel '%s'",
                  wheel_cfg.wheel_name().c_str());

      return wheel_cfg;
    };

    return FourSwerveKinematicsSolverConfig{
      {create_wheel_config(0), create_wheel_config(1), create_wheel_config(2), create_wheel_config(3)},
      SvdLeastSquaresSolverConfig{this->get_parameter("direct_solver.relative_singular_value_threshold")
                                    .get_value<double>(),
                                  this->get_parameter("direct_solver.tikhonov_lambda").get_value<double>()}};
  }

  void FourSwerveKinematicsSolverRos::create_subs_pubs()
  {
    joint_states_sub_ = this->create_subscription<
      sensor_msgs::msg::JointState>(input_joint_states_topic_,
                                    rclcpp::SensorDataQoS(),
                                    std::bind(&FourSwerveKinematicsSolverRos::joint_state_cb,
                                              this,
                                              std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_twist_topic_, 10);

    twist_cmd_sub_ = this->create_subscription<
      geometry_msgs::msg::Twist>(input_twist_topic_,
                                 rclcpp::SystemDefaultsQoS(),
                                 std::bind(&FourSwerveKinematicsSolverRos::twist_cb, this, std::placeholders::_1));

    joint_cmd_pub_ = this->create_publisher<actuator_msgs::msg::Actuators>(output_joint_commands_topic_, 10);
  }

  void FourSwerveKinematicsSolverRos::joint_state_cb(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    const auto t_now_ns{this->now().nanoseconds()};

    std::unordered_map<std::string, std::size_t> joint_name_to_index;

    for(std::size_t i{0}; i < msg->name.size(); ++i)
    {
      joint_name_to_index[msg->name[i]] = i;
    }

    std::array<SteerableWheelState, 4> wheel_states;

    try
    {
      for(std::size_t wheel_index{0}; wheel_index < wheel_paths_.size(); ++wheel_index)
      {
        const auto& wheel{solver_->wheel(wheel_index)};
        const auto& steering_joint_name{wheel.steering_joint().name()};
        const auto& rotation_joint_name{wheel.rotation_joint().name()};

        const auto steering_joint_it{joint_name_to_index.find(steering_joint_name)};
        const auto rotation_joint_it{joint_name_to_index.find(rotation_joint_name)};

        if(steering_joint_it == joint_name_to_index.end())
        {
          throw std::invalid_argument("Steering joint '" + steering_joint_name + "' not found in JointState message.");
        }

        const auto index_steering_joint{steering_joint_it->second};

        if(index_steering_joint >= msg->position.size())
        {
          throw std::invalid_argument("Steering joint '" + steering_joint_name +
                                      "' has no position data in JointState message.");
        }

        if(rotation_joint_it == joint_name_to_index.end())
        {
          throw std::invalid_argument("Rotation joint '" + rotation_joint_name + "' not found in JointState message.");
        }

        const auto index_rotation_joint{rotation_joint_it->second};

        if(index_rotation_joint >= msg->velocity.size())
        {
          throw std::invalid_argument("Rotation joint '" + rotation_joint_name +
                                      "' has no velocity data in JointState message.");
        }

        wheel_states[wheel_index] = SteerableWheelState{wheel.name(),
                                                        RotationJointState{rotation_joint_name,
                                                                           (index_rotation_joint <
                                                                            msg->position.size()) ?
                                                                             msg->position[index_rotation_joint] :
                                                                             0.0,
                                                                           msg->velocity[index_rotation_joint],
                                                                           t_now_ns},
                                                        SteeringJointState{steering_joint_name,
                                                                           msg->position[index_steering_joint],
                                                                           (index_steering_joint <
                                                                            msg->velocity.size()) ?
                                                                             msg->velocity[index_steering_joint] :
                                                                             0.0,
                                                                           t_now_ns}};
      }
    }
    catch(const std::exception& ex)
    {
      RCLCPP_WARN_THROTTLE(joint_state_cb_logger_, *get_clock(), 1000, "%s", ex.what());
      return;
    }

    try
    {
      const Twist twist{solver_->get_twist(wheel_states)};

      if(!received_initial_joint_states_)
      {
        received_initial_joint_states_ = true;
        RCLCPP_INFO(joint_state_cb_logger_, "Received first JointState message. Initial wheel state is now available.");
      }

      geometry_msgs::msg::Twist twist_msg;
      twist_msg.linear.x = twist.vx();
      twist_msg.linear.y = twist.vy();
      twist_msg.linear.z = twist.vz();
      twist_msg.angular.x = twist.wx();
      twist_msg.angular.y = twist.wy();
      twist_msg.angular.z = twist.wz();

      twist_pub_->publish(twist_msg);
    }
    catch(const std::exception& ex)
    {
      RCLCPP_WARN_THROTTLE(joint_state_cb_logger_, *get_clock(), 1000, "%s", ex.what());
      return;
    }
  }

  void FourSwerveKinematicsSolverRos::twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const auto t_now = this->now();

    if(t_now < t_start_pub_commands_ && !received_initial_joint_states_)
    {
      RCLCPP_DEBUG(twist_cb_logger_,
                   "Ignoring twist commands: waiting for first JointState or for waiting time to expire.");
      return;
    }

    try
    {
      Twist twist_cmd{msg->linear.x, msg->linear.y, 0.0, 0.0, 0.0, msg->angular.z, t_now.nanoseconds()};

      const auto wheel_cmds{solver_->get_wheel_commands(twist_cmd)};

      actuator_msgs::msg::Actuators joint_cmds;

      joint_cmds.header.frame_id = "";
      joint_cmds.header.stamp = t_now;
      joint_cmds.position.resize(4);
      joint_cmds.normalized.resize(4);
      joint_cmds.velocity.resize(4);

      for(std::size_t i{0}; i < wheel_cmds.size(); ++i)
      {
        const auto& wheel_cmd{wheel_cmds[i].get()};
        joint_cmds.position[i] = wheel_cmd.steering_joint_command().value();
        joint_cmds.velocity[i] = wheel_cmd.rotation_joint_command().value();
      }

      joint_cmd_pub_->publish(joint_cmds);
    }
    catch(const std::exception& ex)
    {
      RCLCPP_WARN_THROTTLE(twist_cb_logger_, *get_clock(), 1000, "%s", ex.what());
      return;
    }
  }
}  // namespace ground_vehicle_kinematics
