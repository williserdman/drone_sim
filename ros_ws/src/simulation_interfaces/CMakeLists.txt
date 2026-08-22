cmake_minimum_required(VERSION 3.8)
project(simulation_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RunState.msg"
  "msg/FrameMetadata.msg"
  "msg/GroundTruth.msg"
  "msg/ScenarioEvent.msg"
  "msg/ScoreEvent.msg"
  "msg/ArtifactStatus.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
