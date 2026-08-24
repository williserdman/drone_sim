cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(drone_sim_ardupilot_gazebo_plugin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED ARDUPILOT_GAZEBO_SOURCE)
  message(FATAL_ERROR "ARDUPILOT_GAZEBO_SOURCE is required")
endif()

find_package(gz-cmake3 REQUIRED)
gz_find_package(gz-sim8 REQUIRED)
find_package(RapidJSON REQUIRED)

add_library(ArduPilotPlugin SHARED
  ${ARDUPILOT_GAZEBO_SOURCE}/src/ArduPilotPlugin.cc
  ${ARDUPILOT_GAZEBO_SOURCE}/src/SocketUDP.cc
  ${ARDUPILOT_GAZEBO_SOURCE}/src/Util.cc
)
target_include_directories(ArduPilotPlugin PRIVATE
  ${ARDUPILOT_GAZEBO_SOURCE}/include
)
target_link_libraries(ArduPilotPlugin PRIVATE gz-sim8::gz-sim8)
