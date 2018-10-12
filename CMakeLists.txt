cmake_minimum_required(VERSION 2.8.3)
project(camera_is1500)

## Use C++11
#add_definitions(--std=c++11)
#set(__GLIBCXX_USE_CXX11_ABI=0) #added and used in the makefile of the sfmain
#set(CMAKE_CXX_COMPILER "g++-4.8") #compile using g++-4.8 as asked by Thales

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
  tf2_msgs
  #Eigen3
  #Eigen REQUIRED
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
  #INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  INCLUDE_DIRS
  include
#  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    std_msgs
    std_srvs
    nav_msgs
#  DEPENDS #Eigen
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include/camera_is1500
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${GSTREAMER_INCLUDE_DIRS}
                    #${EIGEN3_INCLUDE_DIR}
                    #${Eigen_INCLUDE_DIRS}
                    #lib
)

## Declare a C++ executable
# Example : add_executable(your_node ${${PROJECT_NAME}_SOURCES})

add_executable(camera_is1500 src/camera_is1500_node.cpp)
add_executable(script src/sfhub_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(camera_is1500
  ${catkin_LIBRARIES}
  #lib/libsfaccess.so
  #lib/libinterface.so
  ${CMAKE_CURRENT_LIST_DIR}/lib/libsfaccess.so
  ${CMAKE_CURRENT_LIST_DIR}/lib/libinterface.so
  )


  target_link_libraries(script
    ${catkin_LIBRARIES}
    )

#install(TARGETS talker
#          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})





#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#~ catkin_install_python(PROGRAMS bin/supervisor
  #~ DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(TARGETS
    #list of shared libraries
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  #list of necessary files (xml...)
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)


install(PROGRAMS
   src/waypoint_node.py

   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodes
 )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_supervisor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
