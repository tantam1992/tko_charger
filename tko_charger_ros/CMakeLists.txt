cmake_minimum_required(VERSION 3.0.2)
project(tko_charger_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tko_charger_hw
  tko_charger_interfaces
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES tko_charger_ros
 CATKIN_DEPENDS roscpp tko_charger_hw tko_charger_interfaces
#  DEPENDS system_lib
)

add_library(tko_charger_lib
  src/tko_charger_ros.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tko_charger_node src/tko_charger_node.cpp)
add_dependencies(tko_charger_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tko_charger_node tko_charger_lib ${catkin_LIBRARIES})