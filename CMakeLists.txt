cmake_minimum_required(VERSION 3.8)
project(ground_vehicle_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(angles REQUIRED)

add_library(
  ${PROJECT_NAME}_solvers
  src/solvers/svd_least_squares_solver.cpp
  src/solvers/three_swerve_kinematics.cpp
  src/solvers/four_swerve_kinematics.cpp)
target_link_libraries(${PROJECT_NAME}_solvers PUBLIC Eigen3::Eigen)
target_include_directories(
  ${PROJECT_NAME}_solvers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(
  three_swerve_kinematics_node
  src/three_swerve_kinematics_node.cpp
  src/three_swerve_kinematics_ros.cpp)
target_link_libraries(three_swerve_kinematics_node ${PROJECT_NAME}_solvers)
ament_target_dependencies(three_swerve_kinematics_node rclcpp sensor_msgs
                          geometry_msgs angles actuator_msgs)
target_include_directories(three_swerve_kinematics_node PRIVATE include)

add_executable(
  four_swerve_kinematics_node
  src/four_swerve_kinematics_node.cpp
  src/four_swerve_kinematics_ros.cpp)
target_link_libraries(four_swerve_kinematics_node ${PROJECT_NAME}_solvers)
ament_target_dependencies(four_swerve_kinematics_node rclcpp sensor_msgs
                          geometry_msgs angles actuator_msgs)
target_include_directories(four_swerve_kinematics_node PRIVATE include)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}_solvers three_swerve_kinematics_node four_swerve_kinematics_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_svd_least_squares_solver
                  test/test_svd_least_squares_solver.cpp)
  target_link_libraries(test_svd_least_squares_solver ${PROJECT_NAME}_solvers
                        Eigen3::Eigen)
  target_include_directories(test_svd_least_squares_solver PRIVATE include)

  ament_add_gtest(test_three_swerve_kinematics
                  test/test_three_swerve_kinematics.cpp)
  target_link_libraries(test_three_swerve_kinematics ${PROJECT_NAME}_solvers
                        Eigen3::Eigen)
  target_include_directories(test_three_swerve_kinematics PRIVATE include)

  ament_add_gtest(test_four_swerve_kinematics
                  test/test_four_swerve_kinematics.cpp)
  target_link_libraries(test_four_swerve_kinematics ${PROJECT_NAME}_solvers
                        Eigen3::Eigen)
  target_include_directories(test_four_swerve_kinematics PRIVATE include)

  ament_add_pytest_test(test_launch_contract test/test_launch_contract.py)

  # Register each configurable linter once because ament_lint_auto cannot pass these settings.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cppcheck
    ament_cmake_cpplint
    ament_cmake_flake8)

  ament_cppcheck(include src INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  # Jazzy skips Cppcheck 1.88 and 2.x unless this environment variable enables them.
  set_tests_properties(
    cppcheck PROPERTIES ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")

  ament_cpplint(
    MAX_LINE_LENGTH 120
    FILTERS
      -whitespace/newline
      -whitespace/forcolon
      -build/include_order
      -runtime/indentation_namespace
      -legal/copyright)

  ament_flake8(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ament_flake8.ini")

  # Register the remaining linters declared as test dependencies in package.xml.
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  actuator_msgs
  angles
  Eigen3
  geometry_msgs
  rclcpp
  sensor_msgs)

ament_package()
