cmake_minimum_required(VERSION 2.8.3)
project(hector_exploration_planner)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp rospy dynamic_reconfigure geometry_msgs costmap_2d nav_core pluginlib nav_msgs rosconsole hector_nav_msgs)
find_package(Eigen REQUIRED)

# dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/ExplorationPlanner.cfg)

## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS Eigen
    CATKIN_DEPENDS roscpp rospy dynamic_reconfigure geometry_msgs costmap_2d nav_core pluginlib nav_msgs rosconsole hector_nav_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} hector_exploration_base_global_planner_plugin
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/hector_exploration_planner.cpp)
add_dependencies(${PROJECT_NAME} hector_nav_msgs_gencpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_library(hector_exploration_base_global_planner_plugin src/hector_exploration_base_global_planner_plugin.cpp)
add_dependencies(hector_exploration_base_global_planner_plugin hector_nav_msgs_gencpp)
add_dependencies(hector_exploration_base_global_planner_plugin ${PROJECT_NAME}_gencfg)
target_link_libraries(hector_exploration_base_global_planner_plugin ${PROJECT_NAME} ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
