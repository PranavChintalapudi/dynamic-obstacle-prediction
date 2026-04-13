cmake_minimum_required(VERSION 3.0.2)
project(dynamic_obstacle_prediction)

## C++14 Standard
add_compile_options(-std=c++14)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf
  pcl_ros
  pcl_conversions
)

## Find PCL
find_package(PCL REQUIRED)

## Find Eigen3
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    sensor_msgs 
    nav_msgs 
    geometry_msgs
    visualization_msgs
    tf
    pcl_ros
    pcl_conversions
  DEPENDS 
    PCL
    EIGEN3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

## Obstacle Predictor Node
add_executable(obstacle_predictor_node
  src/ObstaclePredictor.cpp
  src/ObstaclePredictorNode.cpp
)

target_link_libraries(obstacle_predictor_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(obstacle_predictor_node
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(TARGETS obstacle_predictor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)