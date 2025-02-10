cmake_minimum_required(VERSION 3.14)
project(pingpongbotlib VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Manually specify source files
set(SRC_FILES
    src/i2ctest.cpp
    src/i2c.cpp
    src/driver.cpp
)

# Define project include directories
include_directories(pingpongbotlib PRIVATE include)

# Create the executable
add_executable(pingpongbotlib ${SRC_FILES})

# Link required system libraries
target_link_libraries(pingpongbotlib PRIVATE pthread rt)

find_library(I2C_LIB NAMES i2c PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib)

if(NOT I2C_LIB)
    message(FATAL_ERROR "libi2c not found! Install it with: sudo apt install libi2c-dev")
endif()

target_link_libraries(pingpongbotlib PRIVATE ${I2C_LIB})

# Enable warnings
target_compile_options(pingpongbotlib PRIVATE -Wall -Wextra -Wpedantic)

# Install rules (optional)
install(TARGETS pingpongbotlib DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
