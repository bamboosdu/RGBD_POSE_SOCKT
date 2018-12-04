cmake_minimum_required(VERSION 2.8.3)

project(socket_img)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(octomap_DIR /opt/ros/kinetic/share/octomap)


set(gazebo_DIR "/usr/share/gazebo/cmake")

set(OpenCV_DIR "/usr/share/OpenCV")

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roscpp

  OpenCV
  camera_calibration_parsers
  cv_bridge 
  image_transport 
  message_filters 
  nodelet 
  rosconsole 
  octomap
  #Eigen
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES virtual_scan
#  CATKIN_DEPENDS rospy std_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
SET( EIGEN_INCLUDE_DIR /usr/include/eigen3 )
include_directories(
  ${catkin_INCLUDE_DIRS}

  # ${GAZEBO_INCLUDE_DIRS}
  # ${gazebo_msgs_INCLUDE_DIRS}
   ${OpenCV_INCLUDE_DIRS}
  # ${OCTOMAP_INCLUDE_DIRS}
  # ${EIGEN_INCLUDE_DIR}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

#add_dependencies(add_two_ints_server beginner_tutorials_gencpp)
add_executable(sendimg src/socket_server.cpp)
target_link_libraries(sendimg ${catkin_LIBRARIES})
