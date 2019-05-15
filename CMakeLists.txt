cmake_minimum_required(VERSION 2.8.3)
project(objtrack)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(tracking src/tracking.cpp)
target_link_libraries(tracking ${catkin_LIBRARIES} ${PCL_LIBRARIES} modbus)
