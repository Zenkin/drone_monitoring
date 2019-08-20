cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

set(PROJECT quadrocopter_joystick_driver)

project(${PROJECT})

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
    )

add_message_files(
        FILES
        joystick_data.msg
    )

generate_messages(
        DEPENDENCIES
        std_msgs
    )

catkin_package(
        CATKIN_DEPENDS message_runtime
    )

set(SOURCES
        main.cpp
        Joystick.cpp
    )

set(HEADERS
        Joystick.h
    )

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT} ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT} ${catkin_LIBRARIES})
