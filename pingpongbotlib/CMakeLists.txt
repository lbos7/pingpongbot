cmake_minimum_required(VERSION 3.14)
project(pingpongbotlib)

# Find external dependencies
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pingpongbot_msgs REQUIRED)

# Find libi2c if needed
find_library(I2C_LIB NAMES i2c PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib)
if(NOT I2C_LIB)
    message(FATAL_ERROR "libi2c not found! Install it with: sudo apt install libi2c-dev")
endif()

# Create the library
add_library(pingpongbotlib
    src/i2c.cpp
    src/driver.cpp
    src/kinematics.cpp
)

# Link necessary libraries
target_link_libraries(pingpongbotlib
    tf2::tf2
    geometry_msgs::geometry_msgs
    pingpongbot_msgs::pingpongbot_msgs
    pthread
    rt
    ${I2C_LIB}
)

# Make header files available for other packages to use
target_include_directories(pingpongbotlib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # Make headers available during build
    $<INSTALL_INTERFACE:include>  # Make headers available during installation
)

# Ensure C++17 standard
target_compile_features(pingpongbotlib PUBLIC cxx_std_17)

# Install the library and headers
install(TARGETS pingpongbotlib EXPORT pingpongbotlib-targets)
install(DIRECTORY include/pingpongbotlib DESTINATION include)

# Export the CMake targets for other packages to find
install(EXPORT pingpongbotlib-targets
    FILE pingpongbotlib-config.cmake
    NAMESPACE pingpongbotlib::
    DESTINATION lib/cmake/${PROJECT_NAME}
)
