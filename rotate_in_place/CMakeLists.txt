cmake_minimum_required(VERSION 2.8.3)
project(rotate_in_place)

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
  cfg/RotateInPlace.cfg
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pluginlib mbf_msgs mbf_costmap_core tf2_ros mbf_exposed_services dynamic_reconfigure
)

add_library(PID src/pid.cpp)
add_dependencies(PID ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(PID ${catkin_LIBRARIES})

add_library(${PROJECT_NAME} src/rotate_in_place.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} PID)

# install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
# install(FILES rotate_in_place_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
