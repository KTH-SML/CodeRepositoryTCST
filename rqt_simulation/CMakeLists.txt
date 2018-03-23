cmake_minimum_required(VERSION 2.8.3)

project(rqt_simulation)

find_package(catkin REQUIRED
    COMPONENTS roscpp rospy std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(move_base_service_caller src/rqt_simulation/move_base_service_caller.cpp)
target_link_libraries(move_base_service_caller ${catkin_LIBRARIES})

catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
