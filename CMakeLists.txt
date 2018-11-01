cmake_minimum_required(VERSION 2.8.3)
project(setpoint_server)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  mavros
  mavros_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
  SetPoint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  mavros_msgs
  geometry_msgs  
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES setpoint_server
  CATKIN_DEPENDS roscpp mavros mavros_msgs geometry_msgs std_msgs
  #DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sp_node src/main.cpp src/setpoint_server.cpp)
target_link_libraries(sp_node ${catkin_LIBRARIES})