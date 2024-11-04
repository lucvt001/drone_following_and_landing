cmake_minimum_required(VERSION 3.0.2)
project(target_tracking_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  geodesy
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES target_tracking_planner
 CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs geodesy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(relative_ned_position src/relative_ned_position.cpp)
target_link_libraries(relative_ned_position ${catkin_LIBRARIES}) 
add_dependencies(relative_ned_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(offset_relative_ned_position src/offset_relative_ned_position.cpp)
target_link_libraries(offset_relative_ned_position ${catkin_LIBRARIES}) 
add_dependencies(offset_relative_ned_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(gps_to_ned src/gps_to_ned.cpp)
target_link_libraries(gps_to_ned ${catkin_LIBRARIES}) 
add_dependencies(gps_to_ned ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})