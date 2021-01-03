cmake_minimum_required(VERSION 2.8.3)
project(robot_voice)

add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_voice
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(iat_publish 
  src/iat_publish.cpp 
  src/speech_recognizer.c 
  src/linuxrec.c)
target_link_libraries(
   iat_publish
   ${catkin_LIBRARIES} 
   libmsc.so -ldl -lpthread -lm -lrt -lasound)

add_executable(sub_word
  src/sub_word.cpp) 
target_link_libraries(
   sub_word
   ${catkin_LIBRARIES})

add_executable(sub_word_tb3
  src/sub_word_tb3.cpp) 
target_link_libraries(
   sub_word_tb3
   ${catkin_LIBRARIES})




