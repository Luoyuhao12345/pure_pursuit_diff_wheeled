cmake_minimum_required(VERSION 3.0.2)
project(car_sim)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  tf
)

# add_message_files(
#           FILES
#           way_point.msg
#           car_info.msg
#           my_marker.msg
#           way_points.msg
# )

# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )

catkin_package(
  CATKIN_DEPENDS  roscpp rospy std_msgs tf message_runtime 
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(car_sim_gazebo
  src/car_sim_gazebo.cpp
)
add_dependencies(car_sim_gazebo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(car_sim_gazebo
  ${catkin_LIBRARIES}
)