cmake_minimum_required(VERSION 2.8.3)

project(rqt_simulation)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)