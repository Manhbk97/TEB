# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(local_planner)

find_package(catkin REQUIRED
  COMPONENTS
    base_local_planner
    control_toolbox
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    )

  find_package(Eigen3)
  if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(Eigen3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(cfg/RGTPlanner.cfg)

catkin_package(
  CATKIN_DEPENDS
    base_local_planner
    control_toolbox
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros    
    dynamic_reconfigure
  INCLUDE_DIRS include
  LIBRARIES local_planner
)

# build the library
add_library(local_planner
  src/local_planner.cpp
  src/conversions_and_types.cpp
  src/visualization.cpp
  src/trajectory_controller.cpp
)
add_dependencies(local_planner ${local_planner_EXPORTED_TARGETS})

target_link_libraries(local_planner ${catkin_LIBRARIES})




## Build the standalone node executable
add_executable(rgt_planner
  src/local_planner_node.cpp
)

## Specify libraries to link the executable against
target_link_libraries(rgt_planner
  ${catkin_LIBRARIES}
  local_planner
)

## Add dependencies
add_dependencies(rgt_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

install(
  DIRECTORY include/local_planner/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS local_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
  FILES elp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
