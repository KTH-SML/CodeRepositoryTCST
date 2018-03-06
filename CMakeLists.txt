cmake_minimum_required(VERSION 2.8.3)
project(robot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED roscpp geometry_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot
  CATKIN_DEPENDS roscpp geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/robot.cpp
# )

add_executable(${PROJECT_NAME}_node src/robot_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
