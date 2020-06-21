cmake_minimum_required(VERSION 2.8.3)
project(tag_swarm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

#find_package(Eigen3 REQUIRED)

find_package(GTSAM REQUIRED)
#message(STATUS ${GTSAM_INCLUDE_DIR})
#message(STATUS ${GTSAM_VERSION})

find_package(catkin REQUIRED
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  actionlib
  kr_mav_manager
  kr_mav_msgs
  kr_tracker_msgs
  apriltag_msgs
  tf2_ros
  tf2_geometry_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES tag_swarm
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    std_msgs
    kr_mav_manager
    kr_mav_msgs
    kr_tracker_msgs
    apriltag_msgs
    tf2_ros
    tf2_geometry_msgs
    angles
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})

add_executable(tag_manager src/tag_manager.cpp)
target_link_libraries(tag_manager gtsam ${catkin_LIBRARIES})

add_executable(tag_search src/tag_search.cpp)
target_link_libraries(tag_search ${catkin_LIBRARIES})