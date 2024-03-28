cmake_minimum_required(VERSION 3.0.2)
project(template_interfaces)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  TkoCharger.msg
  # Message2.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES template_interfaces
 CATKIN_DEPENDS std_msgs message_runtime rospy roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
