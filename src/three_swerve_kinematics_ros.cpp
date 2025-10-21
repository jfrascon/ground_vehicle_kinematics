#include "ground_vehicle_kinematics/three_swerve_kinematics_ros.hpp"

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

/*
 * ============================================================================
 * FRAME ALIGNMENT ASSUMPTIONS FOR SWERVE DRIVE KINEMATICS
 * ============================================================================
 *
 * This solver assumes a specific kinematic chain configuration that is standard
 * for swerve drive architectures:
 *
 * KINEMATIC CHAIN:
 * steerable_joint:
 *     robot_root_link (e.g base_link) → steerable_link
 * rotation_joint:
 *     steerable_link → wheel_link
 *
 * CRITICAL FRAME ALIGNMENT REQUIREMENTS:
 *
 * 1. STEERABLE JOINT CONFIGURATION:
 *    - steerable_joint connects parent_link (usually base_link) to steerable_link
 *    - steerable_link frame MUST be aligned with parent_link frame
 *    - Translation: free positioning in x, y, z (specifies wheel module location)
 *    - Rotation: MUST be identity (roll = pitch = yaw = 0)
 *    - This means steerable_link x,y,z axes are parallel to parent_link axes
 *
 * 2. ROTATION JOINT CONFIGURATION:
 *    - rotation_joint connects steerable_link to wheel_link
 *    - wheel_link frame MUST be aligned with steerable_link frame
 *    - Translation: only z offset allowed (y = y = 0, z = free for wheel vertical offset)
 *    - Rotation: MUST be identity (roll = pitch = yaw = 0)
 *    - This ensures wheel rotation axis is always vertical (parallel to z)
 *
 * MATHEMATICAL CONSEQUENCE:
 * - Wheel position (x_i,y_i) relative to base_link equals steerable_joint origin
 * - No coordinate transformations needed between frames
 * - Kinematic equations directly use steerable_joint.origin.{x,y} values
 *
 * TYPICAL URDF EXAMPLE:
 * <joint name="steerable_joint_front_left" type="revolute">
 *   <parent link="base_link"/>
 *   <child link="steerable_link_front_left"/>
 *   <origin xyz="0.35 0.25 0.0" rpy="0 0 0"/>  <- ALIGNED FRAMES
 * </joint>
 *
 * <joint name="rotation_joint_front_left" type="continuous">
 *   <parent link="steerable_link_front_left"/>
 *   <child link="wheel_link_front_left"/>
 *   <origin xyz="0 0 -0.05" rpy="0 0 0"/>      <- ALIGNED FRAMES
 * </joint>
 *
 * This configuration is natural for most swerve drive and Ackermann architectures and ensures the kinematic equations
 * remain in the simple 2D form documented in the mathematical derivation.
 */

namespace ground_vehicle_kinematics
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ThreeSwerveKinematicsSolverRos::ThreeSwerveKinematicsSolverRos(const rclcpp::NodeOptions& options):
    rclcpp_lifecycle::LifecycleNode("three_swerve_kinematics_solver", options)
  {
    declare_all_parameters();
    get_all_parameters();
  }

  //////////////////////////////////////////////////////////////////////////////
  // LIFECYCLE CALLBACKS
  //////////////////////////////////////////////////////////////////////////////

  CallbackReturn ThreeSwerveKinematicsSolverRos::on_configure(const rclcpp_lifecycle::State&)
  {
    if(is_ksolver_unknown())
    {
      RCLCPP_ERROR(get_logger(), "Kinematics solver is unknown");
      return CallbackReturn::FAILURE;
    }

    if(is_ksolver_direct())
    {
      create_sub_pub_dk_computations();
    }

    if(is_ksolver_inverse())
    {
      create_sub_pub_ik_computations();
    }

    RCLCPP_INFO(get_logger(), "Solver initialized with type '%s'", ksolver_type_to_cstr());

    return CallbackReturn::SUCCESS;
  }

  //////////////////////////////////////

  CallbackReturn ThreeSwerveKinematicsSolverRos::on_activate(const rclcpp_lifecycle::State&)
  {
    if(is_ksolver_direct())
    {
      twist_pub_->on_activate();
    }

    if(is_ksolver_inverse())
    {
      // Initial joint states subscription and timer are activated each time the node is activated, because if we
      // do it on_configure, then when passing from active to inactive and back to active, we would not re-initialize
      // the inverse kinematics solver with the current wheel states.
      setup_initial_sw_state_subscription();
      joint_states_pub_->on_activate();

      for(auto& [key, pub]: joint_pos_cmd_pubs_)
      {
        pub->on_activate();
      }
    }

    active_ = true;

    RCLCPP_INFO(get_logger(), "Activated");

    return CallbackReturn::SUCCESS;
  }

  //////////////////////////////////////

  CallbackReturn ThreeSwerveKinematicsSolverRos::on_deactivate(const rclcpp_lifecycle::State&)
  {
    active_ = false;

    if(is_ksolver_direct() && twist_pub_)
    {
      twist_pub_->on_deactivate();
    }

    if(is_ksolver_inverse())
    {
      finish_initial_sw_state_reception();

      if(joint_states_pub_)
      {
        joint_states_pub_->on_deactivate();
      }

      for(auto& [key, pub]: joint_pos_cmd_pubs_)
      {
        pub->on_deactivate();
      }
    }

    RCLCPP_INFO(get_logger(), "Deactivated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ThreeSwerveKinematicsSolverRos::on_cleanup(const rclcpp_lifecycle::State&)
  {
    active_ = false;

    // Subscriber and publisher for direct kinematics computations.
    joint_states_sub_.reset();
    twist_pub_.reset();

    // Subscriber and publisher for inverse kinematics computations.
    twist_sub_.reset();
    joint_states_pub_.reset();
    joint_pos_cmd_pubs_.clear();

    // Clean up anything related to inverse kinematics initial state reception.
    reset_prev_sw_states();
    finish_initial_sw_state_reception();

    RCLCPP_INFO(get_logger(), "Cleaned up.");
    return CallbackReturn::SUCCESS;
  }
  //////////////////////////////////////

  CallbackReturn ThreeSwerveKinematicsSolverRos::on_error(const rclcpp_lifecycle::State&)
  {
    active_ = false;
    RCLCPP_ERROR(get_logger(), "Error transition.");
    return CallbackReturn::SUCCESS;
  }

  //////////////////////////////////////

  CallbackReturn ThreeSwerveKinematicsSolverRos::on_shutdown(const rclcpp_lifecycle::State&)
  {
    active_ = false;
    RCLCPP_INFO(get_logger(), "Shutdown.");
    return CallbackReturn::SUCCESS;
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::create_sub_pub_dk_computations()
  {
    // When solving direct kinematics, JointState messages are received and Twist messages are published
    joint_states_sub_.reset();

    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      input_joint_states_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&ThreeSwerveKinematicsSolverRos::joint_state_cb, this, std::placeholders::_1));

    twist_pub_.reset();
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_twist_topic, rclcpp::SystemDefaultsQoS());
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::create_sub_pub_ik_computations()
  {
    twist_sub_.reset();

    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      input_twist_topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&ThreeSwerveKinematicsSolverRos::twist_cb, this, std::placeholders::_1));

    joint_states_pub_.reset();

    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output_joint_states_topic,
                                                                             rclcpp::SystemDefaultsQoS());

    // Publish each joint command individually is something we want in simulation to command each joint controller
    // plugin separately, but it is not needed in real robots.
    if(this->get_parameter("use_sim_time").as_bool())
    {
      // Create individual publishers for each joint.
      for(const auto& sw: sws_)
      {
        const std::string rotation_joint_topic{"joints/" + sw.rotation_joint.name + "/cmd_pos"};

        joint_pos_cmd_pubs_[sw.rotation_joint.name] = this->create_publisher<std_msgs::msg::Float64>(
          rotation_joint_topic,
          rclcpp::SystemDefaultsQoS());

        const std::string steerable_joint_topic{"joints/" + sw.steerable_joint.name + "/cmd_pos"};

        joint_pos_cmd_pubs_[sw.steerable_joint.name] = this->create_publisher<std_msgs::msg::Float64>(
          steerable_joint_topic,
          rclcpp::SystemDefaultsQoS());
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::declare_all_parameters()
  {
    this->declare_parameter<std::string>("robot_prefix", "robot_");

    // Fixed wheel sections - declare all parameters for each section
    const std::vector<std::string> required_sw_sections = {"steerable_wheel_a",
                                                           "steerable_wheel_b",
                                                           "steerable_wheel_c"};

    for(const auto& required_sw_section: required_sw_sections)
    {
      const std::string sw_path{"steerable_wheels." + required_sw_section};

      // Basic wheel parameters
      this->declare_parameter<std::string>(sw_path + ".name");
      this->declare_parameter<double>(sw_path + ".radius");

      // Steerable joint parameters
      this->declare_parameter<std::string>(sw_path + ".steerable_joint.name");
      this->declare_parameter<std::string>(sw_path + ".steerable_joint.parent_link_name");
      this->declare_parameter<std::string>(sw_path + ".steerable_joint.child_link_name");

      // Steerable joint origin
      this->declare_parameter<double>(sw_path + ".steerable_joint.origin.x");
      this->declare_parameter<double>(sw_path + ".steerable_joint.origin.y");
      this->declare_parameter<double>(sw_path + ".steerable_joint.origin.z");
      this->declare_parameter<double>(sw_path + ".steerable_joint.origin.R");
      this->declare_parameter<double>(sw_path + ".steerable_joint.origin.P");
      this->declare_parameter<double>(sw_path + ".steerable_joint.origin.Y");

      // Steerable joint angular limits
      this->declare_parameter<double>(sw_path + ".steerable_joint.angular_limits.lower");
      this->declare_parameter<double>(sw_path + ".steerable_joint.angular_limits.upper");

      // Rotation joint parameters
      this->declare_parameter<std::string>(sw_path + ".rotation_joint.name");
      this->declare_parameter<std::string>(sw_path + ".rotation_joint.parent_link_name");
      this->declare_parameter<std::string>(sw_path + ".rotation_joint.child_link_name");

      // Rotation joint origin
      this->declare_parameter<double>(sw_path + ".rotation_joint.origin.x");
      this->declare_parameter<double>(sw_path + ".rotation_joint.origin.y");
      this->declare_parameter<double>(sw_path + ".rotation_joint.origin.z");
      this->declare_parameter<double>(sw_path + ".rotation_joint.origin.R");
      this->declare_parameter<double>(sw_path + ".rotation_joint.origin.P");
      this->declare_parameter<double>(sw_path + ".rotation_joint.origin.Y");
    }

    // Optional parameters (with default values)
    this->declare_parameter<std::string>("kinematics_solver_type", "mixed");
    this->declare_parameter<double>("inverse_kinematics_solver_initialization_timeout", 10.0);
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::finish_initial_sw_state_reception()
  {
    ik_solver_waiting_for_initial_sw_state_ = false;
    initial_sw_state_sub_.reset();
    initial_sw_state_timer_.reset();
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::get_all_parameters()
  {
    robot_prefix_ = safe_get_parameter<std::string>("robot_prefix");

    // Get optional parameters (will use defaults if not provided)
    const std::string solver_type_str = safe_get_parameter<std::string>("kinematics_solver_type");
    ksolver_type_                     = parse_solver_type(solver_type_str);

    if(is_ksolver_unknown())
    {
      throw std::invalid_argument("Invalid kinematics_solver_type parameter: " + solver_type_str);
    }

    if(is_ksolver_inverse())
    {
      ik_solver_initial_sw_state_reception_timeout_ = safe_get_parameter<double>(
        "inverse_kinematics_solver_initialization_timeout");

      if(ik_solver_initial_sw_state_reception_timeout_ < 0.0)
      {
        throw std::invalid_argument("inverse_kinematics_solver_initialization_timeout must be non-negative.");
      }
    }

    // Load steerable wheel configurations
    get_sw_parameters();
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::get_sw_parameters()
  {
    // Fixed wheel sections that match the steerable_wheels_ array indices
    const std::vector<std::string> required_sw_sections = {"steerable_wheel_a",
                                                           "steerable_wheel_b",
                                                           "steerable_wheel_c"};

    std::unordered_set<std::string> sw_names;
    size_t index{0};

    for(const std::string& sw_section: required_sw_sections)
    {
      const std::string sw_path{"steerable_wheels." + sw_section};

      // Storing directly into the array item.
      SteerableWheel& sw = sws_[index];

      sw.name = robot_prefix_ + safe_get_parameter<std::string>(sw_path + ".name");

      // Validate wheel name is not empty
      if(sw.name.empty())
      {
        throw std::invalid_argument("Wheel name in " + sw_section + " cannot be empty.");
      }

      sw.radius = safe_get_parameter<double>(sw_path + ".radius");

      // Load steerable joint
      sw.steerable_joint.name = robot_prefix_ + safe_get_parameter<std::string>(sw_path + ".steerable_joint.name");

      // Validate steerable joint names are not empty
      if(sw.steerable_joint.name.empty())
      {
        throw std::invalid_argument("Steerable joint name in " + sw_section + " cannot be empty.");
      }

      sw.steerable_joint.parent_link_name = robot_prefix_ + safe_get_parameter<std::string>(
                                                              sw_path + ".steerable_joint.parent_link_name");

      if(sw.steerable_joint.parent_link_name.empty())
      {
        throw std::invalid_argument("Steerable joint parent_link_name in " + sw_section + " cannot be empty.");
      }

      sw.steerable_joint.child_link_name = robot_prefix_ + safe_get_parameter<std::string>(
                                                             sw_path + ".steerable_joint.child_link_name");

      if(sw.steerable_joint.child_link_name.empty())
      {
        throw std::invalid_argument("Steerable joint child_link_name in " + sw_section + " cannot be empty.");
      }

      sw.steerable_joint.origin               = get_pose_parameter(sw_path + ".steerable_joint.origin");
      sw.steerable_joint.angular_limits.lower = safe_get_parameter<double>(sw_path +
                                                                           ".steerable_joint.angular_limits.lower");
      sw.steerable_joint.angular_limits.upper = safe_get_parameter<double>(sw_path +
                                                                           ".steerable_joint.angular_limits.upper");

      // Load rotation joint
      sw.rotation_joint.name = robot_prefix_ + safe_get_parameter<std::string>(sw_path + ".rotation_joint.name");

      // Validate rotation joint names are not empty
      if(sw.rotation_joint.name.empty())
      {
        throw std::invalid_argument("Rotation joint name in " + sw_section + " cannot be empty.");
      }

      sw.rotation_joint.parent_link_name = robot_prefix_ + safe_get_parameter<std::string>(
                                                             sw_path + ".rotation_joint.parent_link_name");

      if(sw.rotation_joint.parent_link_name.empty())
      {
        throw std::invalid_argument("Rotation joint parent_link_name in " + sw_section + " cannot be empty.");
      }

      // Validate kinematic chain consistency: rotation joint parent must be steerable joint child
      if(sw.rotation_joint.parent_link_name != sw.steerable_joint.child_link_name)
      {
        throw std::invalid_argument(
          "In " + sw_section + ": rotation_joint.parent_link_name ('" + sw.rotation_joint.parent_link_name +
          "') must equal steerable_joint.child_link_name ('" + sw.steerable_joint.child_link_name + "').");
      }

      sw.rotation_joint.child_link_name = robot_prefix_ +
                                          safe_get_parameter<std::string>(sw_path + ".rotation_joint.child_link_name");

      if(sw.rotation_joint.child_link_name.empty())
      {
        throw std::invalid_argument("Rotation joint child_link_name in " + sw_section + " cannot be empty.");
      }

      sw.rotation_joint.origin = get_pose_parameter(sw_path + ".rotation_joint.origin");

      // Validate rotation joint pose: only Z translation allowed, no rotation
      const auto& rot_pose{sw.rotation_joint.origin};
      const auto tolerance{1e-6};

      if(std::abs(rot_pose.x) > tolerance || std::abs(rot_pose.y) > tolerance || std::abs(rot_pose.R) > tolerance ||
         std::abs(rot_pose.P) > tolerance || std::abs(rot_pose.Y) > tolerance)
      {
        throw std::invalid_argument("In " + sw_section + ": rotation_joint pose must have x=0, y=0, R=0, P=0, Y=0. " +
                                    "Only z translation is allowed. Got: x=" + std::to_string(rot_pose.x) +
                                    ", y=" + std::to_string(rot_pose.y) + ", R=" + std::to_string(rot_pose.R) +
                                    ", P=" + std::to_string(rot_pose.P) + ", Y=" + std::to_string(rot_pose.Y));
      }

      // Insert wheel name for uniqueness validation
      sw_names.insert(sw.name);

      ++index;
    }

    // Validate that all wheel names are unique
    if(sw_names.size() != 3)
    {
      throw std::invalid_argument("Wheel names must be unique. Found " + std::to_string(sw_names.size()) +
                                  " unique names out of 3.");
    }
  }

  //////////////////////////////////////////////////////////////////////////////

  Pose ThreeSwerveKinematicsSolverRos::get_pose_parameter(const std::string& pose_path) const
  {
    Pose pose;
    pose.x = safe_get_parameter<double>(pose_path + ".x");
    pose.y = safe_get_parameter<double>(pose_path + ".y");
    pose.z = safe_get_parameter<double>(pose_path + ".z");
    pose.R = safe_get_parameter<double>(pose_path + ".R");
    pose.P = safe_get_parameter<double>(pose_path + ".P");
    pose.Y = safe_get_parameter<double>(pose_path + ".Y");

    return pose;
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::initial_sw_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Map for easy access to joints in the JointState message.
    std::unordered_map<std::string, size_t> map_name_index;

    for(size_t i{0}; i < msg->name.size(); ++i)
    {
      map_name_index[msg->name[i]] = i;
    }

    // Validate we have at least 6 joints (3 steerable + 3 rotation).
    // If we do not have all the required joints, we keep receiving messages until timeout.
    if(msg->name.size() < 6)
    {
      RCLCPP_WARN(get_logger(),
                  "Initial JointState message has only %zu joints, but we need at least 6 (3 steerable + 3 rotation). "
                  "Using default values for inverse kinematics initialization.",
                  msg->name.size());
      // Keep receiving messages until timeout.
      return;
    }

    // Create local array to accumulate states.
    // Only swap with prev_sw_states_ if all succeed.
    std::array<SteerableWheelState, 3> local_sw_states{};

    for(size_t index{0}; index < 3; ++index)
    {
      // sw at index i is linked to the sw_state at index i.
      const auto& sw       = sws_[index];
      auto& local_sw_state = local_sw_states[index];

      // Initialize joint names and wheel name first
      local_sw_state.wheel_name                       = sw.name;
      local_sw_state.rotation_joint_state.joint_name  = sw.rotation_joint.name;
      local_sw_state.steerable_joint_state.joint_name = sw.steerable_joint.name;

      // Search the required joints in the JointState message.
      const auto rotation_js_it  = map_name_index.find(sw.rotation_joint.name);
      const auto steerable_js_it = map_name_index.find(sw.steerable_joint.name);

      // Check if the required joints (steering and rotation) were found in the JointState message.
      if(rotation_js_it == map_name_index.end() || steerable_js_it == map_name_index.end())
      {
        RCLCPP_WARN(get_logger(),
                    "Initial JointState message missing entries for joints '%s' and/or '%s'.",
                    sw.rotation_joint.name.c_str(),
                    sw.steerable_joint.name.c_str());
        // Keep receiving messages until timeout.
        return;
      }

      // Check if the found rontation joint has velocity data.
      if(rotation_js_it->second >= msg->velocity.size())
      {
        RCLCPP_WARN(get_logger(),
                    "Initial JointState lacks velocity for rotation joint '%s'.",
                    sw.rotation_joint.name.c_str());
        // Keep receiving messages until timeout.
        return;
      }

      // Check if the found steering joint has position data.
      if(steerable_js_it->second >= msg->position.size())
      {
        RCLCPP_WARN(get_logger(),
                    "Initial JointState lacks position for steering joint '%s'.",
                    sw.steerable_joint.name.c_str());
        // Keep receiving messages until timeout.
        return;
      }

      // Save to local array (only commit to prev_sw_states_ if all wheels complete successfully)
      // Mandatory.
      local_sw_state.rotation_joint_state.alpha = msg->velocity[rotation_js_it->second];
      // Optional.
      local_sw_state.rotation_joint_state.theta = (rotation_js_it->second < msg->position.size()) ?
                                                    msg->position[rotation_js_it->second] :
                                                    0.0;

      // Mandatory.
      local_sw_state.steerable_joint_state.theta = angles::normalize_angle(msg->position[steerable_js_it->second]);
      // Optional.
      local_sw_state.steerable_joint_state.alpha = (steerable_js_it->second < msg->velocity.size()) ?
                                                     msg->velocity[steerable_js_it->second] :
                                                     0.0;

      local_sw_state.steerable_joint_state.steering_saturated = false;
    }

    // All wheels processed successfully, swap local array with prev_sw_states_
    std::swap(*prev_sw_states_, local_sw_states);

    t_prev_sw_states_ = this->now();

    finish_initial_sw_state_reception();

    RCLCPP_INFO(get_logger(), "Loaded inverse solver seed from JointState message.");
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::initial_sw_state_timeout_cb()
  {
    RCLCPP_WARN(get_logger(),
                "Inverse solver did not receive initial JointState within %.2f s. "
                "Resetting to default seed.",
                ik_solver_initial_sw_state_reception_timeout_);

    finish_initial_sw_state_reception();
  }

  //////////////////////////////////////////////////////////////////////////////

  bool ThreeSwerveKinematicsSolverRos::is_ksolver_direct() const noexcept
  {
    return ksolver_type_ == KinematicsSolverType::Direct || ksolver_type_ == KinematicsSolverType::Mixed;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool ThreeSwerveKinematicsSolverRos::is_ksolver_inverse() const noexcept
  {
    return ksolver_type_ == KinematicsSolverType::Inverse || ksolver_type_ == KinematicsSolverType::Mixed;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool ThreeSwerveKinematicsSolverRos::is_ksolver_unknown() const noexcept
  {
    return ksolver_type_ == KinematicsSolverType::Unknown;
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if(!active_)
    {
      return;
    }

    // joint names can be in any order, so we create a map from name to index for fast lookup.
    std::unordered_map<std::string, size_t> joint_name_to_index;

    for(size_t i{0}; i < msg->name.size(); ++i)
    {
      joint_name_to_index[msg->name[i]] = i;
    }

    std::array<SteerableWheelState, 3> sw_states{};
    bool ok{true};

    // Joints defined in the steerable wheels must be present in the JointState message.
    for(size_t index{0}; index < 3; ++index)
    {
      // sw at index i is linked to the sw_state at index i.
      const auto& sw = sws_[index];
      auto& sw_state = sw_states[index];

      // Try to find the index at which the steering and wheel joints are located in the JointState message.
      const auto rotation_js_it = joint_name_to_index.find(sw.rotation_joint.name);
      const auto steering_js_it = joint_name_to_index.find(sw.steerable_joint.name);

      if(rotation_js_it == joint_name_to_index.end() || rotation_js_it->second >= msg->velocity.size())
      {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             1000,
                             "Rotation joint '%s' missing in JointState.",
                             sw.rotation_joint.name.c_str());
        ok = false;
        break;
      }

      // If any of the required joints (steering or rotation) is missing, we cannot compute direct kinematics.
      if(steering_js_it == joint_name_to_index.end() || steering_js_it->second >= msg->position.size())
      {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             1000,
                             "Steering joint '%s' missing in JointState.",
                             sw.steerable_joint.name.c_str());
        ok = false;
        break;
      }

      sw_state.rotation_joint_state.joint_name          = sw.rotation_joint.name;
      sw_state.rotation_joint_state.alpha               = msg->velocity[rotation_js_it->second];
      sw_state.rotation_joint_state.theta               = (rotation_js_it->second < msg->position.size()) ?
                                                            msg->position[rotation_js_it->second] :
                                                            std::numeric_limits<double>::quiet_NaN();
      sw_state.steerable_joint_state.steering_saturated = false;

      sw_state.steerable_joint_state.joint_name = sw.steerable_joint.name;
      sw_state.steerable_joint_state.theta      = angles::normalize_angle(msg->position[steering_js_it->second]);
      sw_state.steerable_joint_state.alpha      = (steering_js_it->second < msg->velocity.size()) ?
                                                    msg->velocity[steering_js_it->second] :
                                                    std::numeric_limits<double>::quiet_NaN();
    }

    if(!ok)
    {
      return;
    }

    auto twist_opt = ThreeSwerveKinematicsSolver::solve_direct(sws_, sw_states);

    if(!twist_opt)
    {
      RCLCPP_WARN_THROTTLE(get_logger(),
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

  const char* ThreeSwerveKinematicsSolverRos::ksolver_type_to_cstr() const noexcept
  {
    switch(ksolver_type_)
    {
      case KinematicsSolverType::Direct:
      {
        return "direct";
      }
      case KinematicsSolverType::Inverse:
      {
        return "inverse";
      }
      case KinematicsSolverType::Mixed:
      {
        return "mixed";
      }
      case KinematicsSolverType::Unknown:
      default:
      {
        return "unknown";
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////

  ThreeSwerveKinematicsSolverRos::KinematicsSolverType ThreeSwerveKinematicsSolverRos::parse_solver_type(
    const std::string& solver_type_str) noexcept
  {
    if(solver_type_str == "direct")
    {
      return KinematicsSolverType::Direct;
    }

    if(solver_type_str == "inverse")
    {
      return KinematicsSolverType::Inverse;
    }

    if(solver_type_str == "mixed")
    {
      return KinematicsSolverType::Mixed;
    }

    return KinematicsSolverType::Unknown;
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::reset_prev_sw_states() noexcept
  {
    prev_sw_states_.reset();
  }

  //////////////////////////////////////////////////////////////////////////////

  template<typename T>
  T ThreeSwerveKinematicsSolverRos::safe_get_parameter(const std::string& name) const
  {
    try
    {
      rclcpp::Parameter parameter;

      if(!this->get_parameter(name, parameter))
      {
        throw std::invalid_argument("Parameter '" + name + "' not found");
      }

      return parameter.get_value<T>();
    }
    catch(const rclcpp::exceptions::InvalidParameterTypeException& ex)
    {
      throw std::invalid_argument("Parameter '" + name + "' has invalid type. Expected: " + typeid(T).name() +
                                  ". Error: " + ex.what());
    }
    catch(const std::exception& ex)
    {
      throw std::invalid_argument("Failed to get parameter '" + name + "': " + ex.what());
    }
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::set_default_prev_sw_states()
  {
    reset_prev_sw_states();
    prev_sw_states_ = std::make_unique<std::array<SteerableWheelState, 3>>();

    // Initialize joint names from the configuration using the index map
    for(size_t index{0}; index < 3; ++index)
    {
      // sw at index i is linked to the prev_sw_state at index i.
      const auto& sw   = sws_[index];
      auto& prev_state = (*prev_sw_states_)[index];

      prev_state.rotation_joint_state.joint_name  = sw.rotation_joint.name;
      prev_state.steerable_joint_state.joint_name = sw.steerable_joint.name;
    }

    t_prev_sw_states_ = this->now();
  }

  //////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::setup_initial_sw_state_subscription()
  {
    // If inverse computations are not considered, then do nothing.
    // If inverse computations are not considered, there is no need to initialize the prev_sw_states_.
    if(!is_ksolver_inverse())
    {
      return;
    }

    // Reset (set default values) the previous wheel states used as seed for the inverse kinematics computations.
    set_default_prev_sw_states();

    // If no timeout is set (zero or negative), then we do not subscribe to an initial JointState message,
    // and we set the default previous wheel states used as seed for the inverse kinematics computations.
    if(ik_solver_initial_sw_state_reception_timeout_ <= 0.0)
    {
      return;
    }

    // If a timeout is set, we also subscribe to an initial JointState message to register the initial steering wheel
    // states to be used in the conversion process twist -> joint states.
    ik_solver_waiting_for_initial_sw_state_ = true;

    initial_sw_state_sub_.reset();

    initial_sw_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      initial_joint_states_topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&ThreeSwerveKinematicsSolverRos::initial_sw_state_cb, this, std::placeholders::_1));

    initial_sw_state_timer_.reset();

    initial_sw_state_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(ik_solver_initial_sw_state_reception_timeout_),
      std::bind(&ThreeSwerveKinematicsSolverRos::initial_sw_state_timeout_cb, this));

    RCLCPP_INFO(get_logger(),
                "Waiting for initial JointState message on topic '%s' to initialize inverse kinematics solver",
                input_joint_states_topic);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////

  void ThreeSwerveKinematicsSolverRos::twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // If inverse computations are not considered, then do nothing.
    // If inverse computations are not considered, twist_cb should not be attended, but just in case.
    if(!is_ksolver_inverse())
    {
      return;
    }

    // If the node is not active, no callback can be processed.
    if(!active_)
    {
      return;
    }

    // If we are still waiting for the initial JointState message to initialize the prev_sw_states_, then keep
    // waiting until the message is received or the timer to receive the message has expired.
    if(ik_solver_waiting_for_initial_sw_state_)
    {
      RCLCPP_DEBUG_THROTTLE(get_logger(),
                            *get_clock(),
                            1000,
                            "Ignoring twist commands until initial JointState seed arrives.");
      return;
    }

    // For a ground vehicle, we only consider vx, vy, and wz. The other components are set to NaN, since
    // they are not used by the solver, and in any case they should be null in a ground vehicle.
    Twist twist;
    twist.vx = msg->linear.x;
    twist.vy = msg->linear.y;
    twist.vz = std::numeric_limits<double>::quiet_NaN();
    twist.wx = std::numeric_limits<double>::quiet_NaN();
    twist.wy = std::numeric_limits<double>::quiet_NaN();
    twist.wz = msg->angular.z;

    auto sw_states{ThreeSwerveKinematicsSolver::solve_inverse(sws_, twist, *prev_sw_states_)};
    const auto t_now{this->now()};

    sensor_msgs::msg::JointState js_msg;
    js_msg.header.stamp = this->now();
    js_msg.header.frame_id.clear();

    js_msg.name.reserve(6);
    js_msg.position.reserve(6);
    js_msg.velocity.reserve(6);

    std_msgs::msg::Float64 rotation_joint_pos_cmd;
    std_msgs::msg::Float64 steerable_joint_pos_cmd;

    for(size_t i{0}; i < sw_states.size(); ++i)
    {
      auto& sw_state{sw_states[i]};
      auto& prev_sw_state{(*prev_sw_states_)[i]};

      js_msg.name.push_back(sw_state.rotation_joint_state.joint_name);
      js_msg.velocity.push_back(sw_state.rotation_joint_state.alpha);  // alpha is present

      // When receiving a Twist msg, we can compute the velocity at which the wheel should turn, but to compute the
      // position of each wheel, we need to know the position of each of them in the previous state and then execute the
      // following expression, for each wheel (integration):
      // rotation_joint.theta_current = rotation_joint.theta_previous + DeltaT * rotation_joint.alpha_previous.
      // This is, the current position is the previous position plus all the rotation performed by the wheel from the
      // previous iteration to this one, i.e, for a time period of DeltaT, at a rotation velocity of alpha_previous,
      // which is the velocity the wheel used from the previous iteration to this one.


      const auto delta_t{(t_now - t_prev_sw_states_).seconds()};
      const auto delta_theta{prev_sw_state.rotation_joint_state.alpha * delta_t};
      // Normalize theta to [0, 2*pi) range to avoid discontinuities.
      sw_state.rotation_joint_state.theta = angles::normalize_angle_positive(prev_sw_state.rotation_joint_state.theta +
                                                                             delta_theta);


      js_msg.position.push_back(sw_state.rotation_joint_state.theta);

      rotation_joint_pos_cmd.data = sw_state.rotation_joint_state.theta;

      js_msg.name.push_back(sw_state.steerable_joint_state.joint_name);
      js_msg.position.push_back(sw_state.steerable_joint_state.theta);  // phi is present

      sw_state.steerable_joint_state.alpha = (sw_state.steerable_joint_state.theta -
                                              prev_sw_state.steerable_joint_state.theta) /
                                             delta_t;
      js_msg.velocity.push_back(sw_state.steerable_joint_state.alpha);

      steerable_joint_pos_cmd.data = sw_state.steerable_joint_state.theta;


      if(sw_state.steerable_joint_state.steering_saturated)
      {
        const auto& limits = sws_[i].steerable_joint.angular_limits;
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             1000,
                             "Steering joint '%s' saturated: commanded %.3f rad within [%.3f, %.3f].",
                             sw_state.steerable_joint_state.joint_name.c_str(),
                             sw_state.steerable_joint_state.theta,
                             limits.lower,
                             limits.upper);
      }

      prev_sw_state = sw_state;

      if(this->get_parameter("use_sim_time").as_bool())
      {
        // In simulation, we publish each joint command individually.
        joint_pos_cmd_pubs_[sw_state.rotation_joint_state.joint_name]->publish(rotation_joint_pos_cmd);
        joint_pos_cmd_pubs_[sw_state.steerable_joint_state.joint_name]->publish(steerable_joint_pos_cmd);
      }
    }

    t_prev_sw_states_ = t_now;
    joint_states_pub_->publish(js_msg);
  }
}  // namespace ground_vehicle_kinematics
