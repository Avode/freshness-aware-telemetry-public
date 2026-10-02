cmake_minimum_required(VERSION 3.10)
project(fleetscope_player_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

add_library(PlayerController SHARED PlayerController.cc)
target_link_libraries(PlayerController PRIVATE
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8)
