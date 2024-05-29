cmake_minimum_required(VERSION 3.0.2)
project(diff-rs-pcl2)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros REQUIRED
  roscpp
  sensor_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES diff-rs-pcl2
  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(D-dbscan-cluster src/dbscan-cluster.cpp)
target_link_libraries(D-dbscan-cluster ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(D-dbscan-cluster ${catkin_EXPORTED_TARGETS})

add_library(D-voxelgrid-cube src/voxelgrid-cube.cpp)
target_link_libraries(D-voxelgrid-cube ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(D-voxelgrid-cube ${catkin_EXPORTED_TARGETS})

add_executable(D-main-a src/main-a.cpp)
target_link_libraries(D-main-a D-voxelgrid-cube D-dbscan-cluster)
add_dependencies(D-main-a ${catkin_EXPORTED_TARGETS})

add_executable(D-main-b src/main-b.cpp)
target_link_libraries(D-main-b D-voxelgrid-cube D-dbscan-cluster)
add_dependencies(D-main-b ${catkin_EXPORTED_TARGETS})

add_executable(D-entrypoint src/entrypoint.cpp)
target_link_libraries(D-entrypoint ${catkin_LIBRARIES})

add_executable(D-safety-ab src/safety-ab.cpp)
target_link_libraries(D-safety-ab ${catkin_LIBRARIES})
add_dependencies(D-safety-ab ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS D-main-a D-main-b D-entrypoint D-safety-ab
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
