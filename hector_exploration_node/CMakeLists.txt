cmake_minimum_required(VERSION 2.8.3)
project(hector_exploration_node)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS hector_exploration_planner hector_nav_msgs)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS hector_exploration_planner hector_nav_msgs
    INCLUDE_DIRS
    LIBRARIES
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(exploration_planner_node src/exploration_node.cpp)

target_link_libraries(exploration_planner_node ${catkin_LIBRARIES})

add_dependencies(exploration_planner_node hector_nav_msgs_gencpp)