cmake_minimum_required(VERSION 3.0.2)
project(aruco_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES aruco_detector
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/*.cpp
# )

add_executable(aruco_detector src/detector.cpp src/camera.cpp src/main.cpp)
target_link_libraries(aruco_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(aruco_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})