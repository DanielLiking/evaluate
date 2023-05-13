cmake_minimum_required(VERSION 3.0.2)
project(evaluate)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(PACKAGE_DEPENDENCIES
  nav_msgs
  roscpp
  rospy
  laser_geometry
  pcl_ros
  tf
  std_msgs
  sensor_msgs
)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(

  CATKIN_DEPENDS
  ${PACKAGE_DEPENDENCIES}

)

#target_include_directories(evaluate SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(evaluate src/Getdata.cpp)
target_link_libraries(evaluate
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   )
