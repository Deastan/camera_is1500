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
    v = overGetData();
    std::cout << v[0] << " " << v[1] << " " << v[2] << " "
    << v[3] << " " << v[4] << " " << v[5] << std::endl;

    sensor_msgs::PointCloud cloud;

    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "camera_is1500";

    cloud.points.resize(1);
    cloud.points[0].x = 5;//v[0];
    cloud.points[0].y = 4;//v[1];
    cloud.points[0].z = 3;//v[2];

    // cloud.channels.resize(1);
    // cloud.channels[0].name = "intensities";
    // cloud.channels[0].values.resize(num_points);
    // for(unsigned int j = 0; j < num_points; ++j){
    //   cloud.points[j].x = ranges[j]*cos(angles[j]);
    //   cloud.points[j].y = ranges[j]*sin(angles[j]);
    //   cloud.points[j].z = 0; //to be handled by the laser transform frame
    //   cloud.channels[0].values[j] = 100;
    // } // end loop

    track_pub.publish(cloud);
    r.sleep();
  } // end loop

  return 0;
} // end main
