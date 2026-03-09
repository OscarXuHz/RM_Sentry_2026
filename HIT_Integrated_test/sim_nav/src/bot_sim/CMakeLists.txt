cmake_minimum_required(VERSION 3.0.2)
project(bot_sim)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf
  serial
  livox_ros_driver2
  dynamic_reconfigure
  teb_local_planner
  costmap_converter
  angles
  sentry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlcib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# ## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   TwoPoints.msg
#   Angles.msg
# )

# ## Generate services in the 'srv' folder
#  add_service_files(
#    FILES
#    RRTStar.srv
#    Astar.srv
#  )

# ## Generate actions in the 'action' folder
#  add_action_files(
#    FILES
#    NavAction.action
#  )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   visualization_msgs
 )
 generate_dynamic_reconfigure_options(
  cfg/test.cfg
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
# LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp rospy std_msgs serial message_runtime dynamic_reconfigure angles sentry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cfg/cpp
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/bot_sim.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(scan_2_node src/scan_2_node.cpp)
add_executable(imu_filter src/imu_filter.cpp)
add_executable(real_robot_transform src/real_robot_transform.cpp)
# add_executable(real_robot_transform_3d src/real_robot_transform_3d.cpp include/bot_sim/EKF.h)
# add_executable(cmd2ser src/cmd2ser.cpp)
# add_executable(ser2msg src/ser2msg.cpp)
add_executable(ser2msg_decision src/ser2msg_decision.cpp)
add_executable(ser2msg_decision_givepoint src/ser2msg_decision_givepoint.cpp)
# add_executable(wavefront src/wavefront.cpp)
# add_executable(test_transform src/test_transform.cpp)
# add_executable(lidar_filter src/lidar_filter.cpp)
add_executable(threeD_lidar_filter src/threeD_lidar_filter.cpp)
add_executable(threeD_lidar_filter_pointcloud src/threeD_lidar_filter_pointcloud.cpp)
add_executable(threeD_lidar_merge src/threeD_lidar_merge.cpp)
add_executable(threeD_lidar_merge_pointcloud src/threeD_lidar_merge_pointcloud.cpp)
add_executable(threeD_lidar_merge_pointcloud_test src/threeD_lidar_merge_pointcloud_test.cpp)
# add_executable(dbscan_bfs src/dbscan_bfs.cpp)
add_executable(dbscan_bfs_3D src/dbscan_bfs_3D.cpp)
# add_executable(threeD_lidar_test src/threeD_lidar_test.cpp)
# add_executable(ser2msg_write_only src/ser2msg_write_only.cpp)
# add_executable(transform_listener src/transform_listener.cpp)
add_executable(dstarlite src/dstarlite.cpp)
add_executable(dwa src/dwa.cpp)
# add_executable(dstarlite src/dstarlite_test_while.cpp)
# add_executable(dstarlite_pointcloud src/dstarlite_pointcloud.cpp)
# add_executable(pc_vel_scaler src/pc_vel_scaler.cpp)

add_executable(calculate_shift_theta src/calculate_shift_theta.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")


## Specify libraries to link a library or executable target against
# target_link_libraries(scan_2_node
# ${catkin_LIBRARIES}
# )
target_link_libraries(imu_filter
  ${catkin_LIBRARIES}
)
target_link_libraries(real_robot_transform
  ${catkin_LIBRARIES}
)
# target_link_libraries(real_robot_transform_3d
#   ${catkin_LIBRARIES}
#   ${GLOG_LIBRARIES}
#   glog::glog
# )
# target_link_libraries(cmd2ser
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(ser2msg
#   ${catkin_LIBRARIES}
# )
target_link_libraries(ser2msg_decision
  ${catkin_LIBRARIES}
)
target_link_libraries(ser2msg_decision_givepoint
  ${catkin_LIBRARIES}
)
# target_link_libraries(wavefront
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(test_transform
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(lidar_filter
#   ${catkin_LIBRARIES}
# )
target_link_libraries(threeD_lidar_filter
  ${catkin_LIBRARIES}
)
target_link_libraries(threeD_lidar_filter_pointcloud
  ${catkin_LIBRARIES}
)
target_link_libraries(threeD_lidar_merge
  ${catkin_LIBRARIES}
)
target_link_libraries(threeD_lidar_merge_pointcloud
  ${catkin_LIBRARIES}
)
target_link_libraries(threeD_lidar_merge_pointcloud_test
  ${catkin_LIBRARIES}
)
# target_link_libraries(dbscan_bfs
#   ${catkin_LIBRARIES}
# )
target_link_libraries(dbscan_bfs_3D
  ${catkin_LIBRARIES}
)
# target_link_libraries(threeD_lidar_test
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(ser2msg_write_only
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(transform_listener
#   ${catkin_LIBRARIES}
# )
target_link_libraries(dstarlite
  ${catkin_LIBRARIES}
)
# target_link_libraries(dstarlite_pointcloud
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(pc_vel_scaler
#   ${catkin_LIBRARIES}
# )
target_link_libraries(calculate_shift_theta
  ${catkin_LIBRARIES}
)

target_link_libraries(dwa
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(real_robot_transform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(threeD_lidar_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(ser2msg_write_only ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(dstarlite ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(dwa ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(dstarlite_pointcloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#############
## Install ##
#############3D_lidar_filteration
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/map_image_publisher.py
  scripts/local_frame_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bot_sim.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
