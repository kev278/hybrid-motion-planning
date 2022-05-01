cmake_minimum_required(VERSION 2.8.3)
project(hybrid_local_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  custom_srvs
)

catkin_package(

)


include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(hybrid_local_planner
  src/hybrid_local_planner.cpp
 )


target_link_libraries(hybrid_local_planner
   ${catkin_LIBRARIES}
 )
install(
  FILES hybrid_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
