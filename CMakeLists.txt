cmake_minimum_required(VERSION 2.8.3)
project(lidar_obstacle_detection)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  visualization_msgs
  geometry_msgs
  tf2_msgs
  tf
  tf_conversions
  eigen_conversions
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -fopenmp")

catkin_package()

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

link_directories(${PCL_LIBRARY_DIRS})

add_library(Lidar_Process
  src/lidar_process.cpp
)

add_executable(${PROJECT_NAME}_node src/main.cpp )
target_link_libraries(${PROJECT_NAME}_node
  Lidar_Process
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
