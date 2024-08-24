cmake_minimum_required(VERSION 3.0.2)
project(data_logging)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

catkin_package(
	#  INCLUDE_DIRS include
  LIBRARIES data_logging
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(inch_data_logging src/inch_data_logging.cpp)
add_dependencies(inch_data_logging ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inch_data_logging ${catkin_LIBRARIES})
