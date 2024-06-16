cmake_minimum_required(VERSION 3.0.2)
project(inch_toolbox)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES inch_toolbox
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(inch_toolbox
  src/inch_workbench.cpp
  src/inch_joint.cpp
  src/inch_misc.cpp  
)

add_dependencies(inch_toolbox ${catkin_EXPORTED_TARGETS})
target_link_libraries(inch_toolbox ${catkin_LIBRARIES})

install(TARGETS inch_toolbox
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)