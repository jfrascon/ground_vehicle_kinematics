#include "ground_vehicle_kinematics/three_swerve_kinematics_ros.hpp"

#include "ground_vehicle_kinematics/core/types.hpp"
#include "ground_vehicle_kinematics/parameter_utils.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "angles/angles.h"
#include "rclcpp/parameter.hpp"

namespace ground_vehicle_kinematics
{
  ThreeSwerveKinematicsSolverRos::ThreeSwerveKinematicsSolverRos(const rclcpp::NodeOptions& options):
    rclcpp::Node("three_swerve_kinematics_solver", options),
    twist_cb_logger_(this->get_logger().get_child("twist_cb")),
    joint_state_cb_logger_(this->get_logger().get_child("joint_state_cb"))
  {
    for(size_t index{0}; index < 3UL; ++index)
    {
      const std::string st_wheel_path{"steerable_wheels.steerable_wheel_" + std::to_string(index)};

      // Basic wheel parameters
      this->declare_parameter<std::string>(st_wheel_path + ".name");
      this->declare_parameter<double>(st_wheel_path + ".radius");

      // Steerable joint parameters
      this->declare_parameter<std::string>(st_wheel_path + ".steerable_joint.name");
      this->declare_parameter<std::string>(st_wheel_path + ".steerable_joint.parent_link_name");
      this->declare_parameter<std::string>(st_wheel_path + ".steerable_joint.child_link_name");

      // Steerable joint origin
      this->declare_parameter<double>(st_wheel_path + ".steerable_joint.origin.x");
      this->declare_parameter<double>(st_wheel_path + ".steerable_joint.origin.y");
      this->declare_parameter<double>(st_wheel_path + ".steerable_joint.origin.z");
      this->declare_parameter<double>(st_wheel_path + ".steerable_joint.origin.R");
      this->declare_parameter<double>(st_wheel_path + ".steerable_joint.origin.P");
      this->declare_parameter<double>(st_wheel_path + ".steerable_joint.origin.Y");

      // Steerable joint angular limits
      this->declare_parameter<double>(st_wheel_path + ".steerable_joint.limits.lower");
      this->declare_parameter<double>(st_wheel_path + ".steerable_joint.limits.upper");

      // Rotation joint parameters
      this->declare_parameter<std::string>(st_wheel_path + ".rotation_joint.name");
      this->declare_parameter<std::string>(st_wheel_path + ".rotation_joint.parent_link_name");
      this->declare_parameter<std::string>(st_wheel_path + ".rotation_joint.child_link_name");

      // Rotation joint origin
      this->declare_parameter<double>(st_wheel_path + ".rotation_joint.origin.x");
      this->declare_parameter<double>(st_wheel_path + ".rotation_joint.origin.y");
      this->declare_parameter<double>(st_wheel_path + ".rotation_joint.origin.z");
      this->declare_parameter<double>(st_wheel_path + ".rotation_joint.origin.R");
      this->declare_parameter<double>(st_wheel_path + ".rotation_joint.origin.P");
      this->declare_parameter<double>(st_wheel_path + ".rotation_joint.origin.Y");

      this->declare_parameter<double>(st_wheel_path + ".rotation_joint.limits.lower");
      this->declare_parameter<double>(st_wheel_path + ".rotation_joint.limits.upper");
    }

    this->declare_parameter<double>("initial_wheel_states_reception_period", 10.0);

    auto initial_wheel_states_reception_period = this->get_parameter("initial_wheel_states_reception_period")
                                                   .get_value<double>();

    if(initial_wheel_states_reception_period > 0.0)
    {
      t_start_pub_commands_ = this->now() + rclcpp::Duration::from_seconds(initial_wheel_states_reception_period);
    }
    else
    {
      RCLCPP_WARN(get_logger(),
                  "Initial wheel states reception period is negative (%.2f). No period used",
                  initial_wheel_states_reception_period);

      t_start_pub_commands_ = this->now();
    }

    // Load steerable wheel configurations
    process_st_wheel_parameters();
    create_subs_pubs();

    RCLCPP_INFO(this->get_logger(), "ThreeSwerveKinematicsSolverRos node initialized.");
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::check_st_wheel_relations()
  {
    // Constant used to compare double values.
    constexpr double epsilon{1e-9};

    // All wheels must have the same radius.
    if(st_wheel_descs_[0].st_wheel.radius != st_wheel_descs_[1].st_wheel.radius ||
       st_wheel_descs_[0].st_wheel.radius != st_wheel_descs_[2].st_wheel.radius)
    {
      throw std::invalid_argument("All steerable wheels must have the same radius. Found: r0 = " +
                                  std::to_string(st_wheel_descs_[0].st_wheel.radius) +
                                  ", r1 = " + std::to_string(st_wheel_descs_[1].st_wheel.radius) +
                                  ", r2 = " + std::to_string(st_wheel_descs_[2].st_wheel.radius));
    }

    // st_wheel_0 lies on the x-axis of the robot's frame, on either side:
    // α0 = 0 -> (x0, y0) = ( l0 * cos(α0), l0 * sin(α0) ) = ( +l0, 0 )
    // α0 = π -> (x0, y0) = ( l0 * cos(α0), l0 * sin(α0) ) = ( -l0, 0 )
    // Since origin.y is a double, we can't compare with exact 0, so we use an epsilon tolerance, so that
    // if -epsilon < origin.y < +epsilon, we consider it to be 0, otherwise, we throw an exception.
    if(std::abs(st_wheel_descs_[0].st_wheel.steerable_joint.origin.y) > epsilon)
    {
      throw std::invalid_argument(
        "steerable_wheel_0 must be located on the x-axis of the robot's frame (i.e., origin.y = 0). Found y = " +
        std::to_string(st_wheel_descs_[0].st_wheel.steerable_joint.origin.y));
    }

    // l0 should not be zero, i.e., the wheel should not be located at the origin of the robot's frame.
    // Again, since origin.x is a double, and we cannot compare with exact 0, we use an epsilon tolerance, so that
    // if -epsilon < origin.x < +epsilon, we consider it to be 0 (depending on the sign of origin.x), and we throw an
    // exception.
    // If l0 is just a little bigger than epsilon, it is programmatically valid, although not physically recommended,
    // but we let it pass, since the question is 'what is the minimum l0 that makes sense physically?', and that depends
    // on the specific robot, and I do not know if there is a general value that can be used as a threshold for all
    // robots.
    if(std::abs(st_wheel_descs_[0].st_wheel.steerable_joint.origin.x) < epsilon)
    {
      throw std::invalid_argument(
        "steerable_wheel_0 must not be located at the origin of the robot's frame (i.e., origin.x != 0). Found x = " +
        std::to_string(st_wheel_descs_[0].st_wheel.steerable_joint.origin.x));
    }

    const double alpha_1{std::atan2(st_wheel_descs_[1].st_wheel.steerable_joint.origin.y,
                                    st_wheel_descs_[1].st_wheel.steerable_joint.origin.x)};

    const double l_1{std::hypot(st_wheel_descs_[1].st_wheel.steerable_joint.origin.x,
                                st_wheel_descs_[1].st_wheel.steerable_joint.origin.y)};

    const double alpha_2{std::atan2(st_wheel_descs_[2].st_wheel.steerable_joint.origin.y,
                                    st_wheel_descs_[2].st_wheel.steerable_joint.origin.x)};

    const double l_2{std::hypot(st_wheel_descs_[2].st_wheel.steerable_joint.origin.x,
                                st_wheel_descs_[2].st_wheel.steerable_joint.origin.y)};

    // l_1 and l_2 must be equal; i.e, the pair of wheels must be equidistant from the robot center.
    if(std::abs(l_1 - l_2) > epsilon)
    {
      throw std::invalid_argument(
        "steerable_wheel_1 and steerable_wheel_2 must be equidistant from the robot center. Found l_1 = " +
        std::to_string(l_1) + ", l_2 = " + std::to_string(l_2));
    }

    // α2 = -α1; i.e, the pair of wheels must be symmetric with respect to the x-axis.
    if(std::abs(alpha_2 + alpha_1) > epsilon)
    {
      throw std::invalid_argument(
        "steerable_wheel_1 and steerable_wheel_2 must be symmetric with respect to the x-axis. Found alpha_1 = " +
        std::to_string(alpha_1) + ", alpha_2 = " + std::to_string(alpha_2));
    }

    // Now that we have validated the symmetry of wheels 1 and 2, we need to check their positions depending on the side
    // where wheel 0 is located.
    // Since we have already validated that st_wheel_1 and st_wheel_2 are symmetric with respect to the x-axis, and
    // are equidistant from the robot center, we only need to check the angle of one of them (say, st_wheel_1),
    // depending on the side where st_wheel_0 is located. If this is correct, then the other one is automatically
    // correct.

    if(st_wheel_descs_[0].st_wheel.steerable_joint.origin.x > 0.0)
    {
      // st_wheel_0 is on the positive x-axis, so st_wheel_1 and st_wheel_2 lie in the half-plane with negative x.
      // st_wheel_1 (y > 0): α1 ∈ [π/2, π),         (x1, y1) = ( l * cos(α1),  l * sin(α1) )
      // st_wheel_2 (y < 0): α2 = -α1 ∈ (-π, -π/2], (x2, y2) = ( l * cos(α2),  l * sin(α2) ) =
      //                                                      = ( l * cos(-α1), l * sin(-α1) ) =
      //                                                      = ( l * cos(α1), -l * sin(α1) ) =

      // Check that α1 is in [π/2, π)
      // By using (M_PI - epsilon) as upper limit, we ensure that α1 cannot be exactly π, which would place
      // st_wheel_1 on the negative x-axis, which is not allowed.
      // Again, we use epsilon tolerance to avoid exact comparisons with doubles.
      // Again, a question arises: should we allow α1 to be just a little lesser than (π - epsilon), i.e., very close to
      // the negative x-axis? From a programmatic point of view, it is valid, although not physically recommended, but
      // we let it pass, since the question is 'what is the minimum angle that makes sense physically?', and that
      // depends on the specific robot, and I do not know if there is a general value that can be used as a threshold
      // for all robots.
      if(alpha_1 < M_PI_2 || alpha_1 >= (M_PI - epsilon))
      {
        throw std::invalid_argument("steerable_wheel_1 must lie in the half-plane with negative x, alpha_1 = [pi/2, "
                                    "pi), when steerable_wheel_0 is on the positive x-axis. Found alpha_1 = " +
                                    std::to_string(alpha_1));
      }
    }
    else
    {
      // st_wheel_0 is on the negative x-axis, so that st_wheel_1 and st_wheel_2 lie in the half-plane with positive x.
      // st_wheel_1 (y > 0): α1 ∈ (0, π/2],        (x1, y1) = ( l * cos(α1),  l * sin(α1) )
      // st_wheel_2 (y < 0): α2 = -α1 ∈ [-π/2, 0), (x2, y2) = ( l * cos(α2),  l * sin(α2) ) =
      //                                                       = ( l * cos(-α1), l * sin(-α1) ) =
      //                                                       = ( l * cos(α1), -l * sin(α1) )

      // Check that α1 is in (0, π/2]
      // By using epsilon as lower limit, we ensure that α1 cannot be exactly 0, which would place
      // st_wheel_1 on the positive x-axis, which is not allowed.
      // Again, we use epsilon tolerance to avoid exact comparisons with doubles.
      // Again, a question arises: should we allow α1 to be just a little bigger than epsilon, i.e., very close to
      // the positive x-axis? From a programmatic point of view, it is valid, although not physically recommended, but
      // we let it pass, since the question is 'what is the minimum angle that makes sense physically?', and that
      // depends on the specific robot, and I do not know if there is a general value that can be used as a threshold
      // for all robots.
      if(alpha_1 < epsilon || alpha_1 > M_PI_2)
      {
        throw std::invalid_argument("steerable_wheel_1 must lie in the half-plane with positive x, alpha_1 = (0, "
                                    "pi/2], when steerable_wheel_0 is on the negative x-axis. Found alpha_1 = " +
                                    std::to_string(alpha_1));
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::create_subs_pubs()
  {
    // When solving direct kinematics, JointState messages are received and Twist messages are published
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      input_joint_states_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ThreeSwerveKinematicsSolverRos::joint_state_cb, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_twist_topic_, 10);

    // When solving inverse kinematics, Twist messages are received and Actuator messages are published
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      input_twist_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&ThreeSwerveKinematicsSolverRos::twist_cb, this, std::placeholders::_1));

    joint_commands_pub_ = this->create_publisher<actuator_msgs::msg::Actuators>(output_joint_commands_topic_, 10);
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Joint names can be in any order in msg, so a map name-index is created for fast lookup.
    std::unordered_map<std::string, size_t> joint_name_to_index;

    for(size_t i{0}; i < msg->name.size(); ++i)
    {
      joint_name_to_index[msg->name[i]] = i;
    }

    bool ok{true};

    // Variable where to store the steerable wheel states received from the JointState message, and if they are
    // valid, store them into st_wheel_descs_.
    std::array<SteerableWheelState, 3> st_wheel_states;

    // All the states to be saved correspond to the current time.
    auto t_now = this->now().nanoseconds();

    // Steerable and rotation joints defined in each st_wheel in st_wheel_descs_ must be present in the JointState
    // message.
    for(size_t i{0}; i < st_wheel_descs_.size(); ++i)
    {
      // st_wheel at index i is linked to the st_wheel_state at index i from construction of the st_wheel_descs_.
      const auto& st_wheel{st_wheel_descs_[i].st_wheel};
      auto& st_wheel_state{st_wheel_states[i]};
      st_wheel_state.wheel_name = st_wheel.name;

      // Search for the steerable and rotation joints in msg.
      const auto steerable_js_it{joint_name_to_index.find(st_wheel.steerable_joint.name)};
      const auto rotation_js_it{joint_name_to_index.find(st_wheel.rotation_joint.name)};

      // Check if the required steerable joint was found in the msg or if it has position data.
      // For a steerable joint, the required data is position. Velocity is not strictly needed.
      // The condition after || checks that even though the joint name was found, it has position data. There is
      // the possibility that the JointState message has the name in the name array, but not enough entries in the
      // position array.
      if(steerable_js_it == joint_name_to_index.end() || steerable_js_it->second >= msg->position.size())
      {
        RCLCPP_WARN_THROTTLE(joint_state_cb_logger_,
                             *get_clock(),
                             1000,
                             "Steerable joint '%s' missing in JointState message or has no position data",
                             st_wheel.steerable_joint.name.c_str());
        ok = false;
        break;
      }

      // Check if the required rotation joint was found in msg and if it has velocity data.
      // For a rotation joint, the required data is angular velocity. Position is not strictly needed.
      // The condition after || checks that even though the joint name was found, it has velocity data. There is
      // the possibility that msg has the name in the name array, but not enough entries in the velocity array.
      if(rotation_js_it == joint_name_to_index.end() || rotation_js_it->second >= msg->velocity.size())
      {
        RCLCPP_WARN_THROTTLE(joint_state_cb_logger_,
                             *get_clock(),
                             1000,
                             "Rotation joint '%s' missing in JointState message or has no velocity data",
                             st_wheel.rotation_joint.name.c_str());
        ok = false;
        break;
      }

      st_wheel_state.steerable_joint_state.joint_name = st_wheel.steerable_joint.name;
      // For a steerable joint, we get position (required) and velocity (optional). Keep raw angle (no normalization)
      // so we respect steering limits even if they extend beyond [-pi, pi].
      st_wheel_state.steerable_joint_state.angle = msg->position[steerable_js_it->second];
      // If velocity data is not available, set to NaN, otherwise use the value from the message.
      st_wheel_state.steerable_joint_state.ang_vel = (steerable_js_it->second < msg->velocity.size()) ?
                                                       msg->velocity[steerable_js_it->second] :
                                                       std::numeric_limits<double>::quiet_NaN();

      st_wheel_state.rotation_joint_state.joint_name = st_wheel.rotation_joint.name;
      // For a rotation joint, we get angular velocity (required) and position (optional).
      st_wheel_state.rotation_joint_state.ang_vel = msg->velocity[rotation_js_it->second];
      // If position data is not available, set to NaN, otherwise keep the raw angle (no normalization).
      st_wheel_state.rotation_joint_state.angle = (rotation_js_it->second < msg->position.size()) ?
                                                    msg->position[rotation_js_it->second] :
                                                    std::numeric_limits<double>::quiet_NaN();
    }

    // If some required joint data was missing, do not proceed.
    // At this point, not all the required steerable wheel states were retrieved from the JointState message, so
    // the st_wheel_states in the st_wheel_descs_ are not updated and direct kinematics is not solved.
    if(!ok)
    {
      return;
    }

    // Now, that all required steerable wheel states have been retrieved from the JointState message,
    // update st_wheel_descs_ with the new states.
    for(size_t i{0}; i < st_wheel_states.size(); ++i)
    {
      // Update the stored st_wheel_state for each steerable wheel.
      st_wheel_descs_[i].st_wheel_state   = st_wheel_states[i];
      st_wheel_descs_[i].t_st_wheel_state = t_now;
    }

    if(!received_initial_joint_states_)
    {
      received_initial_joint_states_ = true;
      RCLCPP_INFO(joint_state_cb_logger_, "Received initial JointState message to seed steerable wheel states.");
    }

    // Solve direct kinematics: (steering angles, wheel velocities) for each wheel -> (robot twist)
    auto twist_opt = ThreeSwerveKinematicsSolver::solve_direct({std::ref(st_wheel_descs_[0].st_wheel),
                                                                std::ref(st_wheel_descs_[1].st_wheel),
                                                                std::ref(st_wheel_descs_[2].st_wheel)},
                                                               {std::ref(st_wheel_descs_[0].st_wheel_state),
                                                                std::ref(st_wheel_descs_[1].st_wheel_state),
                                                                std::ref(st_wheel_descs_[2].st_wheel_state)});

    if(!twist_opt)
    {
      RCLCPP_WARN_THROTTLE(joint_state_cb_logger_,
                           *get_clock(),
                           1000,
                           "solve_direct returned no result (invalid inputs or degenerate configuration)");
      return;
    }

    geometry_msgs::msg::Twist out;
    out.linear.x  = twist_opt->vx;
    out.linear.y  = twist_opt->vy;
    out.linear.z  = twist_opt->vz;
    out.angular.x = twist_opt->wx;
    out.angular.y = twist_opt->wy;
    out.angular.z = twist_opt->wz;

    twist_pub_->publish(out);
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::process_st_wheel_parameters()
  {
    // Set to store names of the processed st_wheels, and to check later that 3 unique names with their configurations
    // were provided.
    std::unordered_set<std::string> st_wheel_names;
    // Small value to compare floating point numbers.

    // Each yaml configuration file must have three sections, each one defining a steerable wheel.
    for(size_t index{0}; index < 3UL; ++index)
    {
      st_wheel_descs_[index] = process_st_wheel_descriptor(*this,
                                                           "steerable_wheels.steerable_wheel_" + std::to_string(index));

      RCLCPP_INFO(get_logger(),
                  "Processed configuration for steerable wheel '%s'",
                  st_wheel_descs_[index].st_wheel.name.c_str());

      // Insert wheel name for uniqueness validation
      st_wheel_names.insert(st_wheel_descs_[index].st_wheel.name);
    }

    // If three wheels are stored, it means all names were unique.
    // If any name was repeated, the size of the set would be less than three, and we would throw and exception
    // explaining the issue.
    if(st_wheel_names.size() != 3)
    {
      throw std::invalid_argument("Wheel names must be unique. Found " + std::to_string(st_wheel_names.size()) +
                                  " unique names out of 3.");
    }

    // Check geometric relations between the three steerable wheels as described in the comment at the beginning of
    // three_swerve_kinematics.cpp
    check_st_wheel_relations();
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const auto t_now = this->now();

    // If we are still waiting for the initial JointState message to initialize the st_wheel_states, then keep waiting
    // until the message is received or the period to receive the initial joint state message has expired.
    if(t_now < t_start_pub_commands_ && !received_initial_joint_states_)
    {
      RCLCPP_DEBUG(twist_cb_logger_, "Ignoring twist commands until initial JointState seed arrives.");
      return;
    }

    // For a ground vehicle, we only consider vx, vy, and wz. The other components are set to NaN, since
    // they are not used by the solver, and in any case they should be null in a ground vehicle.
    Twist twist{msg->linear.x,
                msg->linear.y,
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                msg->angular.z};

    const auto st_wheel_commands{
      ThreeSwerveKinematicsSolver::solve_inverse({std::ref(st_wheel_descs_[0].st_wheel),
                                                  std::ref(st_wheel_descs_[1].st_wheel),
                                                  std::ref(st_wheel_descs_[2].st_wheel)},
                                                 {std::ref(st_wheel_descs_[0].st_wheel_state),
                                                  std::ref(st_wheel_descs_[1].st_wheel_state),
                                                  std::ref(st_wheel_descs_[2].st_wheel_state)},
                                                 twist,
                                                 prev_st_wheel_commands_)};


    const auto t_now_ns{t_now.nanoseconds()};

    actuator_msgs::msg::Actuators joint_commands;

    joint_commands.header.frame_id = "";
    joint_commands.header.stamp    = t_now;
    joint_commands.position.resize(3);
    joint_commands.normalized.resize(3);  // Not used, but resized for consistency.
    joint_commands.velocity.resize(3);

    for(size_t i{0}; i < st_wheel_commands.size(); ++i)
    {
      // st_wheel_command at index i corresponds to st_wheel_descs_ at index i.
      st_wheel_descs_[i].st_wheel_command   = st_wheel_commands[i];
      st_wheel_descs_[i].t_st_wheel_command = t_now_ns;

      // st_wheel_0 is on the x-axis of the robot frame.
      // st_wheel_1 is the one with positive y in the robot frame.
      // st_wheel_2 is the one with negative y in the robot frame.
      joint_commands.position[i] = st_wheel_commands[i].steerable_joint_command.value;
      joint_commands.velocity[i] = st_wheel_commands[i].rotation_joint_command.value;
    }

    joint_commands_pub_->publish(joint_commands);

    // Store the published commands as previous commands for the next iteration.
    prev_st_wheel_commands_ = st_wheel_commands;
  }
}  // namespace ground_vehicle_kinematics
