cmake_minimum_required(VERSION 3.0.2)
project(camera_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_publisher
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs image_transport cv_bridge
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/physical_camera.cpp
# )

add_executable(physical_camera src/physical_camera.cpp)
target_link_libraries(physical_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(physical_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})