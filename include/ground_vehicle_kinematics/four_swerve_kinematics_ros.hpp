#pragma once

/**
 * @file four_swerve_kinematics_ros.hpp
 * @brief ROS2 node for the four-swerve kinematics solver.
 */

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

#include "ground_vehicle_kinematics/solvers/four_swerve_kinematics.hpp"
#include "ground_vehicle_kinematics/types.hpp"

namespace ground_vehicle_kinematics
{
  /**
   * @brief ROS2 node that solves direct and inverse four-swerve kinematics through topics.
   */
  class FourSwerveKinematicsSolverRos: public rclcpp::Node
  {
    public:
      /**
       * @brief Build the ROS node.
       * @param options ROS2 node options.
       * @throws std::invalid_argument If wheel geometry or joint limits are invalid.
       */
      explicit FourSwerveKinematicsSolverRos(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

      /**
       * @brief Process incoming joint states and solve direct kinematics.
       * @param msg Joint-state message.
       */
      void joint_state_cb(const sensor_msgs::msg::JointState::ConstSharedPtr msg);

      /**
       * @brief Process incoming twist commands and solve inverse kinematics.
       * @param msg Twist command message.
       */
      void twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg);

    private:
      /** @brief Input topic for wheel/joint states. */
      static constexpr const char* input_joint_states_topic_{"joint_states"};

      /** @brief Output topic for solved chassis twist. */
      static constexpr const char* output_twist_topic_{"twist"};

      /** @brief Input topic for twist commands. */
      static constexpr const char* input_twist_topic_{"cmd_vel"};

      /** @brief Output topic for solved wheel commands. */
      static constexpr const char* output_joint_commands_topic_{"joint_commands/base"};

      /**
       * @brief YAML paths of the four wheels in the fixed order used by this node.
       */
      // Wheel 0 is front left. Wheel 1 is front right. Wheel 2 is rear left. Wheel 3 is rear right.
      inline static const std::array<std::string, 4> wheel_paths_{
        "steerable_wheels.steerable_wheel_0",
        "steerable_wheels.steerable_wheel_1",
        "steerable_wheels.steerable_wheel_2",
        "steerable_wheels.steerable_wheel_3",
      };

      /** @brief Subscription to joint states. */
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

      /** @brief Subscription to twist commands. */
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_cmd_sub_;

      /** @brief Publisher for solved chassis twist. */
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

      /** @brief Publisher for solved wheel/joint commands. */
      rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr joint_cmd_pub_;

      /** @brief Four-swerve solver used by this node. */
      std::unique_ptr<FourSwerveKinematicsSolver> solver_;

      /** @brief Time at which twist commands start being accepted. */
      rclcpp::Time t_start_pub_commands_;

      /** @brief True once the first valid JointState message has been received. */
      bool received_initial_joint_states_{false};

      /** @brief Logger for the node construction phase. */
      rclcpp::Logger constructor_logger_;

      /** @brief Logger for solver configuration building. */
      rclcpp::Logger create_solver_config_logger_;

      /** @brief Logger for twist callback. */
      rclcpp::Logger twist_cb_logger_;

      /** @brief Logger for joint-state callback. */
      rclcpp::Logger joint_state_cb_logger_;

      /**
       * @brief Build the four-swerve solver configuration from ROS parameters.
       * @return Four-swerve solver configuration.
       */
      FourSwerveKinematicsSolverConfig create_solver_config() const;

      /**
       * @brief Create ROS subscriptions and publishers.
       */
      void create_subs_pubs();
  };

}  // namespace ground_vehicle_kinematics
