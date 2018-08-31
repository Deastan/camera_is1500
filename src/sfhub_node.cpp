/*
 * GOAL : Launch script
*/

#include "ros/ros.h"
#include <sstream>
#include <ros/console.h>
//#include "interface_sf_ros_node.h"
#include <boost/bind.hpp>
#include <stdio.h>
#include <ros/callback_queue.h>

int main(int argc, char **argv)
{
  std::cout << "Begin script" << std::endl;

  system("/home/jonathan/ros_catkin_kyb/src/camera_is1500/src/scriptSHub.bash");

  std::cout << "Finish script" << std::endl;

  ros::spin();
  return 0;


} // end main
