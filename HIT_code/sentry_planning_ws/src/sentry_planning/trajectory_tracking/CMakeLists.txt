cmake_minimum_required(VERSION 3.0.2)
project(tracking_node)

set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    std_msgs
    geometry_msgs
    nav_msgs
    message_generation
    trajectory_generation
    sentry_msgs
    ocs2_mpc
    ocs2_core
    ocs2_oc
    ocs2_sqp
    ocs2_robotic_assets
    ocs2_robotic_tools
    OpenCV REQUIRED
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})


include_directories(
    SYSTEM
    include
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES tracking_node
)


set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_executable( tracking_node
    src/tracking_node.cpp
    src/tracking_manager.cpp
    src/local_planner.cpp
    src/KMF.cpp
    src/visualization_utils.cpp
    src/RM_GridMap.cpp
    src/ocs2_sentry/dynamics/SentryRobotDynamtics.cpp
    src/ocs2_sentry/constraint/SentryRobotStateInputConstraint.cpp
    src/ocs2_sentry/constraint/SentryRobotCollisionConstraint.cpp
    src/ocs2_sentry/cost/SentryRobotQuadraticTrackingCost.cpp
    src/ocs2_sentry/SentryRobotInterface.cpp
    )

add_executable(hit_bridge
    src/hit_bridge.cpp
)


target_link_libraries(tracking_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

target_link_libraries(hit_bridge
    ${catkin_LIBRARIES}
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})