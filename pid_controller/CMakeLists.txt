cmake_minimum_required(VERSION 3.0.2)
project(pid_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pid_controller
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ROS executables ##
## Edit this section if you have more code to compile ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Add an executable for main.cpp
add_executable(follow_relative_ned src/follow_relative_ned.cpp)
target_link_libraries(follow_relative_ned ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(follow_relative_ned ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###########
## Installation for library ##
## Do not edit anything below here! ##
###########

## Declare a C++ library
add_library(${PROJECT_NAME} src/pid.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Install the header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

