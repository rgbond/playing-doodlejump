cmake_minimum_required(VERSION 2.8.3)
project(game)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
  rospy
  image_transport
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  DoobotAction.msg
  PlayCtl.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
   ros_numpy
   score
)
