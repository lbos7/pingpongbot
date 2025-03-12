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
find_package(apriltag_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(rosidl_runtime_cpp REQUIRED)



# Add the executable
add_executable(tracker src/tracker.cpp)
ament_target_dependencies(tracker rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV realsense2)

# add_executable(apriltagHandler src/apriltagHandler.cpp)
# ament_target_dependencies(tracker rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# Link the RealSense library explicitly
target_link_libraries(tracker ${OpenCV_LIBS} realsense2)  # Ensure that the RealSense library is linked


# Install
install(TARGETS
    tracker
    # apriltagHandler
    DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME})

include_directories(
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS})

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
