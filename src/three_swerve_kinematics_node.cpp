#include "ground_vehicle_kinematics/three_swerve_kinematics_ros.hpp"
#include <rclcpp/rclcpp.hpp>
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ground_vehicle_kinematics::ThreeSwerveKinematicsSolverRos>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
