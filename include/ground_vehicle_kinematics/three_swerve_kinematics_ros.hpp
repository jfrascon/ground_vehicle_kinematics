#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <actuator_msgs/msg/actuators.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "ground_vehicle_kinematics/core/three_swerve_kinematics.hpp"
#include "ground_vehicle_kinematics/core/types.hpp"
#include "ground_vehicle_kinematics/parameter_utils.hpp"

namespace ground_vehicle_kinematics
{

  /// Lifecycle-aware ROS wrapper that exposes the three-module swerve solver over topics.
  class ThreeSwerveKinematicsSolverRos: public rclcpp::Node
  {
    public:
    explicit ThreeSwerveKinematicsSolverRos(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);

    void twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg);

    private:
    // Message 'JointStates' on topic 'iput/joint_states' is received and message 'Twist' is published on
    // topic 'output/twist'.
    static constexpr const char* input_joint_states_topic_{"mobile_base_kinematics/joint_states"};
    static constexpr const char* output_twist_topic_{"mobile_base_kinematics/twist"};

    // Message 'Twist' on topic 'input/twist' is received and message JointStates is published on topic
    // 'output/joint_states'
    static constexpr const char* input_twist_topic_{"mobile_base_kinematics/twist_cmd"};
    static constexpr const char* output_joint_commands_topic_{"mobile_base_kinematics/joint_commands"};

    // Subscriptions and publishers.
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr joint_commands_pub_;

    std::array<SteerableWheelDescriptor, 3> st_wheel_descs_;
    std::array<SteerableWheelCommand, 3> prev_st_wheel_commands_;

    // Prefix to use with joint names.
    std::string robot_prefix_;

    rclcpp::Time t_start_pub_commands_;
    bool received_initial_joint_states_{false};

    // Loggers (initialized in ctor initializer list)
    rclcpp::Logger twist_cb_logger_;
    rclcpp::Logger joint_state_cb_logger_;

    void check_st_wheel_relations();

    void create_subs_pubs();

    void process_st_wheel_parameters();
  };

}  // namespace ground_vehicle_kinematics
