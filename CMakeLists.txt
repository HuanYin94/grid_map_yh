cmake_minimum_required(VERSION 2.8.3)
project(grid_map_yh)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  geometry_msgs
  sensor_msgs
  cv_bridge
  octomap_msgs
  filters
  libpointmatcher_ros
  pcl_ros
)

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

find_package(libpointmatcher REQUIRED)

find_package(octomap REQUIRED)

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
 # INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
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
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIR}
)

## Declare a cpp executable

add_executable(
  test_demo
  src/test_node.cpp
)

add_executable(
  pointsToMap
  src/pointsToMap.cpp
)

add_executable(
  traversalCheck
  src/traversalCheck.cpp
)

add_executable(
  laserToMap
  src/laserToMap.cpp
)

add_executable(
  updateMap
  src/updateMap.cpp
)

add_executable(
  nowMap
  src/nowMap.cpp
)

add_executable(
  pathPlanning
  src/pathPlanning.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  test_demo
  ${catkin_LIBRARIES}
  ${libpointmatcher_LIBRARIES}
)

target_link_libraries(
  pointsToMap
  ${catkin_LIBRARIES}
  ${libpointmatcher_LIBRARIES}
)

target_link_libraries(
  traversalCheck
  ${catkin_LIBRARIES}
  ${libpointmatcher_LIBRARIES}
)

target_link_libraries(
  laserToMap
  ${catkin_LIBRARIES}
  ${libpointmatcher_LIBRARIES}
)

target_link_libraries(
  updateMap
  ${catkin_LIBRARIES}
  ${libpointmatcher_LIBRARIES}
)

target_link_libraries(
  nowMap
  ${catkin_LIBRARIES}
  ${libpointmatcher_LIBRARIES}
)

target_link_libraries(
  pathPlanning
  ${catkin_LIBRARIES}
  ${libpointmatcher_LIBRARIES}
)

# Mark executables and/or libraries for installation
install(
  TARGETS test_demo pointsToMap traversalCheck laserToMap updateMap nowMap pathPlanning
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY cfg launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

