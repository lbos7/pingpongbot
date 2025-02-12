cmake_minimum_required(VERSION 3.14)
project(pingpongbotlib)

find_library(I2C_LIB NAMES i2c PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib)

if(NOT I2C_LIB)
    message(FATAL_ERROR "libi2c not found! Install it with: sudo apt install libi2c-dev")
endif()

target_link_libraries(pingpongbotlib PRIVATE ${I2C_LIB})

find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pingpongbot_msgs REQUIRED)

add_library(pingpongbotlib
    src/i2c.cpp
    src/driver.cpp
    src/kinematics.cpp
)
target_link_libraries(pingpongbotlib
    tf2::tf2
    pingpongbot_msgs::pingpongbot_msgs
    geometry_msgs::geometry_msgs
    pthread
    rt
    ${I2C_LIB}
)
target_include_directories(pingpongbotlib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(pingpongbotlib PUBLIC -Wall -Wextra -Wpedantic)
target_compile_features(minimeclib PUBLIC cxx_std_17)

install(DIRECTORY include/pingpongbotlib DESTINATION include)

install(TARGETS pingpongbotlib EXPORT pingpongbotlib-targets)

install(EXPORT pingpongbotlib-targets
    FILE pingpongbotlib-config.cmake
    NAMESPACE pingpongbotlib::
    DESTINATION lib/cmake/${PROJECT_NAME}
)
