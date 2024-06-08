# Collect all source files except main.cpp
file(GLOB_RECURSE SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")
list(REMOVE_ITEM SOURCES "${PROJECT_SOURCE_DIR}/src/main.cpp")

# Create a library from the source files
add_library(CarAssistLib ${SOURCES})

# Link OpenCV libraries
target_link_libraries(CarAssistLib ${OpenCV_LIBS})

# Create executable for the main application
add_executable(CarAssist "${PROJECT_SOURCE_DIR}/src/main.cpp")
target_link_libraries(CarAssist CarAssistLib ${OpenCV_LIBS})
