cmake_minimum_required(VERSION 3.0.2)
project(mbf_exposed_services)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            costmap_2d
            mbf_costmap_core
            mbf_msgs
            geometry_msgs
            mbf_abstract_nav
            mbf_utility
            mbf_costmap_nav            
        )


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES mbf_exposed_services
        CATKIN_DEPENDS
            roscpp
            costmap_2d
            mbf_costmap_core
            mbf_msgs
            geometry_msgs
            mbf_abstract_nav
            mbf_utility
            mbf_costmap_nav   
      )
      
include_directories(
      include
      ${catkin_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME} src/check_pose.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)