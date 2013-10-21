cmake_minimum_required(VERSION 2.8.3)
project(hector_exploration_gateway)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs hector_exploration_planner hector_nav_msgs)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS actionlib actionlib_msgs hector_exploration_planner hector_nav_msgs
    INCLUDE_DIRS
    LIBRARIES
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(exploration_planner_gateway src/exploration_gateway.cpp)
add_dependencies(exploration_planner_gateway hector_nav_msgs_gencpp)
target_link_libraries(exploration_planner_gateway ${catkin_LIBRARIES})
