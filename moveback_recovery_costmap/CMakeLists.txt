cmake_minimum_required(VERSION 2.8.3)
project(moveback_recovery_costmap)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  tf2_ros
  costmap_2d
  mbf_msgs
  mbf_costmap_core
  pluginlib
  mbf_exposed_services
  dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/MVBCKCostmap.cfg
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pluginlib mbf_msgs mbf_costmap_core tf2_ros mbf_exposed_services dynamic_reconfigure
)

add_library(${PROJECT_NAME} src/moveback_recovery_costmap.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
# install(FILES moveback_recovery_costmap_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
