cmake_minimum_required(VERSION 2.8.3)
project(doobot)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
  rospy
  game
)

catkin_package(
   CATKIN_DEPENDS message_runtime
   game
)
