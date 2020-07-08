cmake_minimum_required(VERSION 2.8.3)
project(a1_gazebo)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
    a1_msgs
    pluginlib
    nodelet
)

find_package(gazebo REQUIRED)

catkin_package(
    LIBRARIES laserscan_to_pointcloud pointcloud_to_laserscan
    CATKIN_DEPENDS 
    a1_msgs 
    pluginlib
)

include_directories(
    # include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Declare a C++ library
add_library(${PROJECT_NAME}
    src/body.cpp 
)

add_dependencies(${PROJECT_NAME} a1_msgs_gencpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${EXTRA_LIBS}
)

add_library(A1FootContactPlugin SHARED plugin/foot_contact_plugin.cc)
target_link_libraries(A1FootContactPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(A1DrawForcePlugin SHARED plugin/draw_force_plugin.cc)
target_link_libraries(A1DrawForcePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(a1_external_force src/exe/external_force.cpp)
target_link_libraries(a1_external_force ${catkin_LIBRARIES})

add_executable(a1_servo src/exe/servo.cpp)
target_link_libraries(a1_servo ${PROJECT_NAME} ${catkin_LIBRARIES})


