cmake_minimum_required(VERSION 2.8.3)
project(pose)

add_definitions(-std=c++11)
add_definitions(-DUSE_CAFFE)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
                    roscpp rviz
                    message_generation
                    message_runtime
                    std_msgs
                    sensor_msgs
                    geometry_msgs
                    image_transport
                    ## Uncomment these if required
                    cv_bridge
                    tf
                    )

## Uncomment if using OpenCV
## OpenCV
find_package(OpenCV)
find_package(CUDA)
find_package(gflags REQUIRED)
find_package(OpenPose)

# QT4 required for RViz plugins
#find_package(Qt4 REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#   LIBRARIES
   CATKIN_DEPENDS message_runtime std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
##    including locations and requirements for Qt
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS}
)
## Declaring all Nodes

add_executable(openPoseTracker
    src/openPoseTracker.cpp
)

add_executable(initaliser
    src/initaliser.cpp
)

add_executable(simplefollow
    src/simple_follow.cpp
)


## Specify libraries to link a library or executable target against

target_link_libraries(openPoseTracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OpenPose_LIBS}
	gflags
)

target_link_libraries(initaliser
  ${catkin_LIBRARIES}
)

target_link_libraries(simplefollow
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
