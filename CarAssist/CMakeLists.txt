cmake_minimum_required(VERSION 3.10)
project(CarAssist)

set(CMAKE_CXX_STANDARD 14)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Print OpenCV include directories for debugging
message(STATUS "OpenCV include directories: ${OpenCV_INCLUDE_DIRS}")

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Add subdirectories
add_subdirectory(src)
add_subdirectory(test)
