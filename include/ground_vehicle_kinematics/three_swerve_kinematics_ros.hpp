#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "ground_vehicle_kinematics/core/three_swerve_kinematics.hpp"
#include "ground_vehicle_kinematics/core/types.hpp"

namespace ground_vehicle_kinematics
{

  /// Lifecycle-aware ROS wrapper that exposes the three-module swerve solver over topics.
  class ThreeSwerveKinematicsSolverRos: public rclcpp_lifecycle::LifecycleNode
  {
    public:
    explicit ThreeSwerveKinematicsSolverRos(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State&);

    private:
    // Message 'JointStates' on topic 'iput/joint_states' is received and message 'Twist' is published on
    // topic 'output/twist'.
    static constexpr const char* input_joint_states_topic{"joint_states"};
    static constexpr const char* output_twist_topic{"mobile_base_kinematics/twist"};

    // Message 'Twist' on topic 'input/twist' is received and message JointStates is published on topic
    // 'output/joint_states'
    static constexpr const char* input_twist_topic{"cmd_vel"};
    static constexpr const char* output_joint_states_topic{"mobile_base_kinematics/joint_commands"};

    // To compute joint_states based on a twist, we need a previous state to disambiguate solutions, since same
    // velocity yield multiple solutions for steering angles. This topic is used to receive an initial seed for the
    // inverse kinematics solver.
    static constexpr const char* initial_joint_states_topic{"mobile_base_kinematics/initial_joint_states"};

    enum class KinematicsSolverType
    {
      Unknown = 0,
      Direct  = 1,
      Inverse = 2,
      Mixed   = 3
    };

    // Subscriptions and publishers.
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr initial_sw_state_sub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> twist_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>> joint_states_pub_;
    // Individual publisher for each joint position.
    // Useful if commanding joints individually is needed.
    std::unordered_map<std::string, std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>>>
      joint_pos_cmd_pubs_;

    // Timers.
    rclcpp::TimerBase::SharedPtr initial_sw_state_timer_;

    KinematicsSolverType ksolver_type_{KinematicsSolverType::Unknown};

    // sws -> (s)teerable (w)heel(s) configuration:
    std::array<SteerableWheel, 3> sws_;

    // Previous wheel states used as the continuity seed for the inverse solver.
    std::unique_ptr<std::array<SteerableWheelState, 3>> prev_sw_states_;

    // Timestamp needed to integrate the position of the rotation joint for each wheel.
    // When receiving a Twist msg, we can compute the velocity at which the wheel should turn, but to compute the
    // position of each wheel, we need to know the position of each of them in the previous state and then execute the
    // following expression, for each wheel (integration):
    // rotation_joint.theta_current = rotation_joint.theta_previous + DeltaT * rotation_joint.alpha_previous.
    // This is, the current position is the previous position plus all the rotation performed by the wheel from the
    // previous iteration to this one, i.e, for a time period of DeltaT, at a rotation velocity of alpha_previous,
    // which is the velocity the wheel used from the previous iteration to this one.
    rclcpp::Time t_prev_sw_states_;

    bool active_{false};
    bool ik_solver_waiting_for_initial_sw_state_{false};
    double ik_solver_initial_sw_state_reception_timeout_{-1.0};

    // Prefix to use with joint names.
    std::string robot_prefix_;

    void create_sub_pub_dk_computations();

    void create_sub_pub_ik_computations();

    static KinematicsSolverType parse_solver_type(const std::string& value) noexcept;

    void declare_all_parameters();

    void finish_initial_sw_state_reception();

    void get_all_parameters();

    Pose get_pose_parameter(const std::string& pose_path) const;

    void get_sw_parameters();

    void initial_sw_state_timeout_cb();

    bool is_ksolver_direct() const noexcept;

    bool is_ksolver_inverse() const noexcept;

    bool is_ksolver_unknown() const noexcept;

    void reset_prev_sw_states() noexcept;

    template<typename T>
    T safe_get_parameter(const std::string& name) const;

    void set_default_prev_sw_states();

    void setup_initial_sw_state_subscription();

    const char* ksolver_type_to_cstr() const noexcept;

    void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
    void twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void initial_sw_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  };

}  // namespace ground_vehicle_kinematics
