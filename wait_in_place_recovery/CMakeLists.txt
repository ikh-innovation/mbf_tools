cmake_minimum_required(VERSION 3.0.2)
project(wait_in_place_recovery)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  tf2_ros
  costmap_2d
  mbf_msgs
  mbf_costmap_core
  pluginlib
  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pluginlib mbf_msgs mbf_costmap_core tf2_ros
)

add_library(${PROJECT_NAME} src/wait_in_place_recovery.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

