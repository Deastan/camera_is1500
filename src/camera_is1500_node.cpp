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

// from the sfStudio
#ifdef WIN32
#include "windows.h"
#endif
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <unistd.h>
#include "camera_is1500/libsfaccess.h"

using namespace std;

// add the second part
// Return status;
#define ERR 0
#define OK 1

// Sleep specified number of ms
#ifdef WIN32
#define SNOOZE_MS(ms) Sleep(ms)
#else // Linux
#define SNOOZE_MS(ms) usleep((ms) * 1000)
#endif

// Convert radians to degrees
#define RAD2DEG(rad) ((rad) * 180.0 / 3.1415927)

// Stop program?
static bool stop = false;

// Signal handler for ctrl-c
void sigHandler(int sig)
{
    cout << endl << "Stop signal received" << endl;
    stop = true;
}

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
    // test if all work
    ROS_INFO("Salut c'est Joli!");
    //chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  // ros::spin();
  return 0;
} // end main
