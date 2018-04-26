cmake_minimum_required(VERSION 2.8.3)
project(hybrid_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED roscpp message_generation formula_parser)
find_package(Boost REQUIRED)
find_package(Armadillo REQUIRED)

add_message_files(
  FILES
  HybridState.msg
  ControlInput.msg
  CriticalEvent.msg
  Params.msg
  Robustness.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES PPC
  CATKIN_DEPENDS message_runtime formula_parser
#  DEPENDS 
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${formula_parser_INCLUDE_DIRS}
)

add_library(PPC src/PPC.cpp)
target_link_libraries(PPC
  -lopenblas
  -llapack
  ${ARMADILLO_LIBRARIES}
)

add_library(PFC src/PFC.cpp)
target_link_libraries(PFC
  -lopenblas
  -llapack
  ${ARMADILLO_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/hybrid_controller_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  -lopenblas
  -llapack
  ${ARMADILLO_LIBRARIES}
  PPC
  ${formula_parser_LIBRARIES}
  PFC
)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
