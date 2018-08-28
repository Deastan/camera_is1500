cmake_minimum_required(VERSION 2.8.3)
project(camera_is1500)

## Use C++11
#add_definitions(--std=c++11)
set(__GLIBCXX_USE_CXX11_ABI=0) #added and used in the makefile of the sfmain
set(CMAKE_CXX_COMPILER "g++-4.8") #compile using g++-4.8 as asked by Thales

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    std_srvs
#  DEPENDS
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #lib
)

## Declare a C++ executable
# Example : add_executable(your_node ${${PROJECT_NAME}_SOURCES})

add_executable(talker src/camera_is1500_node.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(talker
  ${catkin_LIBRARIES}
  #/home/jonathan/ros_catkin_kyb/camera_is1500/lib/libsfaccess.so
  )

#install(TARGETS talker
#          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
