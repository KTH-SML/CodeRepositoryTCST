cmake_minimum_required(VERSION 2.8.3)
project(formula_parser)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES formula_parser
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} SHARED
  src/exprtk.cpp
#  include/exprtk.hpp
)
