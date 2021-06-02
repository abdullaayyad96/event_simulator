cmake_minimum_required(VERSION 2.8.3)
project(event_simulator)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  message_generation
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES URX
#  CATKIN_DEPENDS geometry_msgs rospy
#  DEPENDS python-numpy python-sys pythin-urx python-math
)


# Install the nodelet library
cs_install()

