cmake_minimum_required(VERSION 3.10)
project(camera)

set(CMAKE_CXX_STANDARD 17)

# Find OpenCV
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUSE_DIRS} src)

# Add the executable and source files
add_executable(camera
    src/detect_marker.cpp
)

# Link OpenCV libraries
target_link_libraries(camera ${OpenCV_LIBS})