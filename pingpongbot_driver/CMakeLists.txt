cmake_minimum_required(VERSION 3.8)
project(pingpongbot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pingpongbot_msgs REQUIRED)

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)

# Try to find the wiringPi headers
find_path(WIRINGPI_INCLUDE_DIRS NAMES wiringPi.h)

# Ensure wiringPi was found
if(NOT WIRINGPI_LIBRARIES)
  message(FATAL_ERROR "Could not find WiringPi library")
endif()

if(NOT WIRINGPI_INCLUDE_DIRS)
  message(FATAL_ERROR "Could not find WiringPi header")
endif()


add_executable(driver src/driver.cpp src/interface.cpp)
ament_target_dependencies(driver rclcpp pingpongbot_msgs std_msgs std_srvs)
target_include_directories(driver PRIVATE ${WIRINGPI_INCLUDE_DIRS})
target_link_libraries(driver ${WIRINGPI_LIBRARIES})

target_include_directories(driver
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # during build
  $<INSTALL_INTERFACE:include>  # during install
)

install(TARGETS
  driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
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