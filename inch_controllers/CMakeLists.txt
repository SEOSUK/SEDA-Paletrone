cmake_minimum_required(VERSION 3.0.2)
project(inch_controllers)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  inch_toolbox
)

add_message_files(
  FILES
)

add_service_files(
  FILES
  admittance.srv
  FextYFilter.srv
  FextZFilter.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs inch_toolbox
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


add_executable(inch_manipulator_control src/inch_manipulator_control.cpp)
add_dependencies(inch_manipulator_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inch_manipulator_control inch_toolbox ${catkin_LIBRARIES})
