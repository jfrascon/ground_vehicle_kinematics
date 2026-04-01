#include "ground_vehicle_kinematics/four_swerve_kinematics_ros.hpp"

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<ground_vehicle_kinematics::FourSwerveKinematicsSolverRos> node;
  int ret{0};

  try
  {
    node = std::make_shared<ground_vehicle_kinematics::FourSwerveKinematicsSolverRos>();
    rclcpp::spin(node);
  }
  catch(const std::exception& ex)
  {
    const auto logger = node ? node->get_logger() : rclcpp::get_logger("four_swerve_kinematics_node");
    RCLCPP_FATAL(logger, "%s.", ex.what());
    ret = 1;
  }

  rclcpp::shutdown();
  return ret;
}
