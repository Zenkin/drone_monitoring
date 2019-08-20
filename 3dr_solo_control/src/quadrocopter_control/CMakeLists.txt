cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

project(quadrocopter_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  quadrocopter_joystick_driver
  roscpp
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES quadrocopter_control
#  CATKIN_DEPENDS geometry_msgs mavros_msgs quadrocopter_joystick_driver roscpp
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(quadrocopter_control_node src/quadrocopter_control_node.cpp src/auPX4.cpp src/auPX4.h)


target_link_libraries(quadrocopter_control_node
  ${catkin_LIBRARIES}
)

