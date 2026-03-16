#include "ground_vehicle_kinematics/three_swerve_kinematics_ros.hpp"

#include <memory>
#include <exception>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  // Start ROS before creating the node.
  rclcpp::init(argc, argv);

  // Keep the pointer nullable so we can still log errors if construction fails.
  std::shared_ptr<ground_vehicle_kinematics::ThreeSwerveKinematicsSolverRos> node;
  int ret{0};

  try
  {
    // Build the node and spin until shutdown.
    node = std::make_shared<ground_vehicle_kinematics::ThreeSwerveKinematicsSolverRos>();
    rclcpp::spin(node);
  }
  catch(const std::exception& ex)
  {
    // If node construction failed, use a plain fallback logger.
    const auto logger = node ? node->get_logger() : rclcpp::get_logger("three_swerve_kinematics_node");
    RCLCPP_FATAL(logger, "%s.", ex.what());
    ret = 1;
  }

  // Always shut ROS down before exiting.
  rclcpp::shutdown();
  return ret;
}
