cmake_minimum_required(VERSION 3.0.2)
project(pih_docking)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  tf2_ros
  costmap_2d
  mbf_msgs
  std_msgs
  mbf_costmap_core
  pluginlib
  ikh_ros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pluginlib mbf_msgs mbf_costmap_core tf2_ros ikh_ros_msgs
)

add_library(${PROJECT_NAME} src/pih_docking.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
# install(FILES moveback_recovery_costmap_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})