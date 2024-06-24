cmake_minimum_required(VERSION 3.0.2)
project(myrobo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

# catkin_python_setup()

catkin_package()
include_directories( ${catkin_INCLUDE_DIRS} )

add_executable(odom_omni src/odom_omni.cpp)
target_link_libraries(odom_omni ${catkin_LIBRARIES})

add_executable(odom_omni_test src/odom_omni_test.cpp)
target_link_libraries(odom_omni_test ${catkin_LIBRARIES})


