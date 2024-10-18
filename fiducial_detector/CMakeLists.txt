cmake_minimum_required(VERSION 3.0.2)
project(fiducial_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fiducial_detector)
 CATKIN_DEPENDS roscpp sensor_msgs std_msgs image_transport cv_bridge
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

add_executable(fiducial_detector src/main.cpp src/detector.cpp)
target_link_libraries(fiducial_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(fiducial_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})