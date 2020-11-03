cmake_minimum_required(VERSION 3.0.2)
project(sensor_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# setup.py is called during catkin build
#catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS rospy
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

