cmake_minimum_required(VERSION 3.5)
project(decision_node)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    message_generation
    serial
    tf2
    tf2_ros
    tf2_geometry_msgs
)

find_package(behaviortree_cpp_v3 REQUIRED)

# 自定义消息
# add_message_files(DIRECTORY msg FILES Decision.msg)
# generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs)

catkin_package(
    INCLUDE_DIRS include
    # LIBRARIES decision_node
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs serial tf2 tf2_ros tf2_geometry_msgs
    DEPENDS BehaviorTree
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

add_executable(strategy_node
    src/strategy_node.cpp
    src/central_occupiable.cpp
    src/motion_change.cpp
    src/recover_change.cpp
    src/chase.cpp
)

add_executable(continuous_forwarder src/test.cpp)

add_executable(mcu_communicator src/mcu_communicator.cpp)

add_executable(command_test src/command_test.cpp)

target_link_libraries(strategy_node
    ${catkin_LIBRARIES}
    ${behaviortree_cpp_v3_LIBRARIES}
)

target_link_libraries(continuous_forwarder ${catkin_LIBRARIES})

target_link_libraries(mcu_communicator ${catkin_LIBRARIES})

target_link_libraries(command_test ${catkin_LIBRARIES})

# 安装规则 mcu_communicator
install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS strategy_node continuous_forwarder mcu_communicator command_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

