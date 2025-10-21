cmake_minimum_required(VERSION 3.8)
project(ground_vehicle_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(angles REQUIRED)

add_library(${PROJECT_NAME}_core src/core/three_swerve_kinematics.cpp)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_core angles)
target_include_directories(
  ${PROJECT_NAME}_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

add_executable(three_swerve_kinematics_node src/three_swerve_kinematics_node.cpp
                                            src/three_swerve_kinematics_ros.cpp)
target_link_libraries(three_swerve_kinematics_node ${PROJECT_NAME}_core)
ament_target_dependencies(three_swerve_kinematics_node rclcpp rclcpp_lifecycle
                          sensor_msgs geometry_msgs angles)
target_include_directories(three_swerve_kinematics_node PRIVATE include)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(
  TARGETS ${PROJECT_NAME}_core three_swerve_kinematics_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Turn on as needed: ament_lint_auto_find_test_dependencies()
endif()

ament_package()
