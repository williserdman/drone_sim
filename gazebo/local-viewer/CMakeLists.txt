cmake_minimum_required(VERSION 3.22)
project(drone_sim_read_only_inspector)

find_package(Qt5 REQUIRED COMPONENTS Core Qml Quick)
find_package(gz-gui8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED COMPONENTS gui)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(ReadOnlyInspector SHARED
  read_only_inspector.cc
  read_only_inspector.qrc
)
target_link_libraries(ReadOnlyInspector PRIVATE
  gz-gui8::gz-gui8
  gz-plugin2::register
  gz-sim8::gui
  Qt5::Core
  Qt5::Qml
  Qt5::Quick
)

