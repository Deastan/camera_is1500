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




  ros::init(argc, argv, "interface_sf_ros_node");

  // init. publisher
  ros::NodeHandle nh;
  ros::Publisher track_pub = nh.advertise<sensor_msgs::PointCloud>("track_camera", 50);
  // declaration of the variables
  std::vector<float> v;
  overInit();

  ros::Rate r(40);

  while(nh.ok())
  {
    // v is a table of float with the data of the IMU of the camera
    // organised as roll   pitch    yaw   posx   posy   posz
    v = overGetData();
    std::cout << v[0] << " " << v[1] << " " << v[2] << " "
    << v[3] << " " << v[4] << " " << v[5] << std::endl;

    //init. message as a PointCloud and publish it
    sensor_msgs::PointCloud cloud;

    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "camera_is1500";

    cloud.points.resize(2);
    // organised as roll  pitch yaw
    cloud.points[0].x = 5;//v[0];
    cloud.points[0].y = 5;//v[1];
    cloud.points[0].z = 5;//v[2];
    // organised as posx  posy posz
    cloud.points[1].x = 4;//v[3];
    cloud.points[1].y = 4;//v[4];
    cloud.points[1].z = 4;//v[5];

    track_pub.publish(cloud);
    r.sleep();
  } // end loop

  return 0;
} // end main
