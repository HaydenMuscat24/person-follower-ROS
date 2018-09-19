cmake_minimum_required(VERSION 2.8.3)
project(stalker)

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

# QT4 required for RViz plugins
#find_package(Qt4 REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

##Generate messages in the 'msg' folder
#add_message_files(
#   FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

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
)
#INCLUDE(${QT_USE_FILE})
#ADD_DEFINITIONS(${QT_DEFINITIONS})
#ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

## Add generated MOC Qt header files
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/comp3431_starter/rviz/**.hpp)
#QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})


## GUI Specific Libraries
#add_library(comp3431_starter_rviz_library
#    src/rviz/controlPanel.cpp
#
#    ${QT_MOC_HPP}
#)

## Declaring all Nodes

add_executable(simple_follow
    src/simple_follow.cpp
)
## Specify libraries to link a library or executable target against

target_link_libraries(simple_follow
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
#  PATTERN ".svn" EXCLUDE
#)

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
