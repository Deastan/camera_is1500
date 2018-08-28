/*
 * Copyright 2018 Jonathan Burkhard, Kyburz S.A., Freienstein Switzerland
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
#include "std_msgs/String.h"


int main(int argc, char **argv)
{
  // ros::init(argc, argv, "interface_sf_ros_node");
  // ROS_INFO("Salut c'est Joli!");
  // ros::Rate loop_rate(10);
  ros::init(argc, argv, "interface_sf_ros_node");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std::cout << "hello world " << count;

    ROS_INFO("Salut c'est Joli!");
    //chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  // ros::spin();
  return 0;
} // end main
