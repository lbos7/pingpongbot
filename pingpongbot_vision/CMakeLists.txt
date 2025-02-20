cmake_minimum_required(VERSION 3.8)
project(pingpongbot_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# Add the executable
add_executable(tracker src/tracker.cpp)

# Link the RealSense library explicitly
target_link_libraries(tracker realsense2)  # Ensure that the RealSense library is linked

# Create executable
ament_target_dependencies(tracker
  rclcpp 
  sensor_msgs 
  geometry_msgs
  cv_bridge 
  OpenCV
  realsense2)

# Install
install(TARGETS tracker
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    models
    DESTINATION share/${PROJECT_NAME})

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
