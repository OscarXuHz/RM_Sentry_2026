cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generation)

set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)

find_package(catkin REQUIRED COMPONENTS
    sentry_msgs
    roscpp 
    rospy
    std_msgs
    nav_msgs
    visualization_msgs 
    gazebo_msgs
    message_generation
    OpenCV REQUIRED
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# 添加服务信息
add_message_files(DIRECTORY msg FILES trajectoryPoly.msg)
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    include
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")  # 没开向量化加速

add_executable( trajectory_generation
    src/TopoSearch.cpp
    src/trajectory_generator_node.cpp
    src/plan_manager.cpp
    src/replan_fsm.cpp
    src/Astar_searcher.cpp
    src/path_smooth.cpp
    src/reference_path.cpp
    src/RM_GridMap.cpp
    src/visualization_utils.cpp
)

target_link_libraries(trajectory_generation
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)
