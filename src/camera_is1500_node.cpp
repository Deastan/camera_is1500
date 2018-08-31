/*
 * GOAL : is1500 camera data acquisition
*/

#include "ros/ros.h"
#include <sstream>
#include <ros/console.h>
//#include "interface_sf_ros_node.h"
#include <boost/bind.hpp>
#include <stdio.h>
#include <ros/callback_queue.h>

// messages
#include <string>
#include <sensor_msgs/PointCloud.h>

// include library
#include "interface.h"



int main(int argc, char **argv)
{
  // declaration of the variables
  std::vector<float> v;


  overInit();
  ros::init(argc, argv, "interface_sf_ros_node");
  // ros::NodeHandle nh;
  // ros::Publisher track_pub = nh.advertise<sensor_msgs::PointCloud>("track_camera", 50);
while(ros::ok())
{
  v = overGetData();
  std::cout << v[0] << " " << v[1] << " " << v[2] << " "
  << v[3] << " " << v[4] << " " << v[5] << std::endl;
}

return 0;
} // end main
