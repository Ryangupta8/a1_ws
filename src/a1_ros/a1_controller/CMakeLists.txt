cmake_minimum_required(VERSION 2.8.3)
project(a1_controller)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    realtime_tools
    a1_msgs
)

catkin_package(
    CATKIN_DEPENDS
    a1_msgs
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

link_directories($(catkin_LIB_DIRS) lib)

add_library(a1_joint_controller src/joint_controller.cpp)
add_dependencies(a1_joint_controller a1_msgs_gencpp)
target_link_libraries(a1_joint_controller ${catkin_LIBRARIES} a1_control_tool)
