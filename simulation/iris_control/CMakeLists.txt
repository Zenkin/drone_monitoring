cmake_minimum_required(VERSION 2.8.3)
project(drone_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    mavros_msgs
    geometry_msgs
    nav_msgs
)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(
	control_node 
	src/main.cpp 
	src/Quadrocopter.cpp 
) 

add_executable(joy_node
    src/joy_control.cpp
) 

target_link_libraries(control_node ${catkin_LIBRARIES})
target_link_libraries(joy_node ${catkin_LIBRARIES})
