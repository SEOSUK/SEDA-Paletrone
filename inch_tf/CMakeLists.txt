cmake_minimum_required(VERSION 3.0.2)
project(inch_tf)

find_package(catkin REQUIRED COMPONENTS
  inch_toolbox
  roscpp
  rospy
  tf2
  tf2_ros
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS inch_toolbox roscpp rospy tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(inch_tf2_broadcaster src/inch_tf2_broadcaster.cpp)
add_dependencies(inch_tf2_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inch_tf2_broadcaster inch_toolbox ${catkin_LIBRARIES})

add_executable(inch_tf2_listener src/inch_tf2_listener.cpp)
add_dependencies(inch_tf2_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inch_tf2_listener ${catkin_LIBRARIES})

add_executable(inch_tf2_listener_global_cmd src/inch_tf2_listener_global_cmd.cpp)
add_dependencies(inch_tf2_listener_global_cmd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inch_tf2_listener_global_cmd ${catkin_LIBRARIES})

add_executable(inch_tf2_listener_global_ref src/inch_tf2_listener_global_ref.cpp)
add_dependencies(inch_tf2_listener_global_ref ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inch_tf2_listener_global_ref ${catkin_LIBRARIES})

add_executable(inch_tf2_listener_global_meas src/inch_tf2_listener_global_meas.cpp)
add_dependencies(inch_tf2_listener_global_meas ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inch_tf2_listener_global_meas ${catkin_LIBRARIES})
