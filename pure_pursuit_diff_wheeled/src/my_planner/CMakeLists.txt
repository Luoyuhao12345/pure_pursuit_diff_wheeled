cmake_minimum_required(VERSION 3.0.2)
project(my_planner)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  tf
)

add_message_files(
          FILES
          way_point.msg
          way_points.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS  roscpp rospy std_msgs tf message_runtime 
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
 include
  ${catkin_INCLUDE_DIRS}
)
