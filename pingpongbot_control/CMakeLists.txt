cmake_minimum_required(VERSION 3.8)
project(pingpongbot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pingpongbot_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(odomUpdate src/odomUpdate.cpp src/kinematics.cpp)
ament_target_dependencies(odomUpdate rclcpp pingpongbot_msgs tf2 tf2_ros sensor_msgs nav_msgs pingpongbot_msgs geometry_msgs)

add_executable(jointStateUpdate src/jointStateUpdate.cpp)
ament_target_dependencies(jointStateUpdate rclcpp sensor_msgs pingpongbot_msgs)

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp geometry_msgs tf2 tf2_ros)

install(TARGETS
  odomUpdate
  jointStateUpdate
  controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  include/
  DESTINATION include/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
