# Find OpenCV package
find_package(OpenCV REQUIRED)

# Locate GTest
find_package(GTest REQUIRED)

# Locate Threads package
find_package(Threads REQUIRED)

# Include directories
include_directories(${GTEST_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/src)

# Add test executable and link with necessary libraries
add_executable(test_image_processing "${PROJECT_SOURCE_DIR}/test/test_image_processing.cpp")
target_link_libraries(test_image_processing CarAssistLib ${GTEST_LIBRARIES} ${OpenCV_LIBS} Threads::Threads)
