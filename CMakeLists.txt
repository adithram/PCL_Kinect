cmake_minimum_required(VERSION 2.8.3)
project(obstacle_avoid)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_ros
  roscpp
  sensor_msgs
  image_geometry
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions

# Find PCL Components
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
    
# Find OpenCV components
find_package( OpenCV REQUIRED )

# Dynamic Reconfigure
generate_dynamic_reconfigure_options(
  cfg/ObstacleAvoid.cfg
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 # INCLUDE_DIRS include
#  LIBRARIES obstacle_avoid
#  CATKIN_DEPENDS cv_bridge image_transport pcl_conversion pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  cfg
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable

# Declaring obstacle depth node, linking libraries
add_executable(obstacle_depth_node src/obstacle_depth_node.cpp src/segmentation.cpp)

# Add Dynamic Reconfigure
add_dependencies(obstacle_depth_node ${PROJECT_NAME}_gencfg)

target_link_libraries(obstacle_depth_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

# declaring pure point cloud node
#add_executable(obstacle_pc_node src/obstacle_pc_node.cpp src/pc_background_subtract.cpp src/Obstacle.cpp)

#target_link_libraries(obstacle_pc_node
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#  ${PCL_LIBRARIES}
#)

# declaring converter node
add_executable(converter_node src/converter_node.cpp)

target_link_libraries(converter_node  
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

