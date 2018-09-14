cmake_minimum_required(VERSION 2.8.3)
project(plotter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED rospy hybrid_controller geometry_msgs)


# find_package(Boost REQUIRED COMPONENTS system)


# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plotter
  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)


include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

catkin_install_python(
  PROGRAMS src/plotter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
