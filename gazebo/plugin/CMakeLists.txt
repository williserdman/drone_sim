cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(drone_sim_ardupilot_gazebo_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_ARDUPILOT_PLUGIN "Build the pinned ArduPilot Gazebo plugin" ON)

find_package(gz-cmake3 REQUIRED)
gz_find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(RapidJSON REQUIRED)

if(BUILD_ARDUPILOT_PLUGIN)
  if(NOT DEFINED ARDUPILOT_GAZEBO_SOURCE)
    message(FATAL_ERROR "ARDUPILOT_GAZEBO_SOURCE is required")
  endif()
  add_library(ArduPilotPlugin SHARED
    ${ARDUPILOT_GAZEBO_SOURCE}/src/ArduPilotPlugin.cc
    ${ARDUPILOT_GAZEBO_SOURCE}/src/SocketUDP.cc
    ${ARDUPILOT_GAZEBO_SOURCE}/src/Util.cc
  )
  target_include_directories(ArduPilotPlugin PRIVATE
    ${ARDUPILOT_GAZEBO_SOURCE}/include
  )
  target_link_libraries(ArduPilotPlugin PRIVATE gz-sim8::gz-sim8)
endif()

add_library(cwru_payload_command_coordinator SHARED
  PayloadCommandCoordinator.cc
)
target_link_libraries(cwru_payload_command_coordinator PRIVATE
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

add_library(drone_sim_detachable_joint_system SHARED
  vendor/gz_sim_8_11_0/DetachableJoint.cc
)
target_link_libraries(drone_sim_detachable_joint_system PRIVATE
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

include(CTest)
if(BUILD_TESTING)
  add_executable(payload_command_coordinator_integration_test
    test/PayloadCommandCoordinatorIntegrationTest.cc
  )
  target_link_libraries(payload_command_coordinator_integration_test PRIVATE
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
  )
  add_test(
    NAME payload_command_coordinator_integration_test
    COMMAND payload_command_coordinator_integration_test
      $<TARGET_FILE:cwru_payload_command_coordinator>
      $<TARGET_FILE:drone_sim_detachable_joint_system>
  )
  set_tests_properties(payload_command_coordinator_integration_test PROPERTIES
    TIMEOUT 30
  )

  add_executable(detachable_joint_contact_integration_test
    test/DetachableJointContactIntegrationTest.cc
  )
  target_link_libraries(detachable_joint_contact_integration_test PRIVATE
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
  )
  add_test(
    NAME detachable_joint_contact_integration_test
    COMMAND detachable_joint_contact_integration_test
      $<TARGET_FILE:drone_sim_detachable_joint_system>
  )
  set_tests_properties(detachable_joint_contact_integration_test PROPERTIES
    TIMEOUT 30
  )
endif()
