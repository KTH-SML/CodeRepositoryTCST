cmake_minimum_required(VERSION 2.8.3)
project(rqt_simulation_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Edge.msg
  Roi.msg
  Sense.msg
  TemporaryTask.msg

  )

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
