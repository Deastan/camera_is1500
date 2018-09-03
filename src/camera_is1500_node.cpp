/*
 * GOAL : is1500 camera data acquisition
*/

#include "ros/ros.h"
#include <sstream>
#include <ros/console.h>
#include <boost/bind.hpp>
#include <stdio.h>
#include <ros/callback_queue.h>

// messages
#include <string>
#include <sensor_msgs/PointCloud.h>

// include library
#include "interface.h"

// Convert radians to degrees
#define RAD2DEG(rad) ((rad) * 180.0 / 3.1415927)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interface_sf_ros_node");

  // init. publisher
  ros::NodeHandle nh;
  ros::Publisher track_pub = nh.advertise<sensor_msgs::PointCloud>("track_camera", 1000);

  ros::Rate loop_rate(10);
  //SfAccess sfa;

  // Open buffer to take data from the camera,
  // Read sfaccess.ini file and open tracker interface
  if (!overInit())
  {
      std::cout << "Error: Failed to open - Whhaat ? "  << std::endl;
      return 0;
  }

  // Loop
  std::vector<float> v;

  while(nh.ok())
  {
    // v is a table of float with the data of the IMU of the camera
    // organised as roll   pitch    yaw   posx   posy   posz
    v = overGetData();
    // // Screen data
    // std::cout << v[0] << " " << v[1] << " " << v[2] << " "
    // << v[3] << " " << v[4] << " " << v[5] << std::endl;

    //init. message as a PointCloud and publish it
    sensor_msgs::PointCloud cloud;

    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "camera_is1500";

    cloud.points.resize(2);
    // organised as roll  pitch yaw
    cloud.points[0].x = v[0];
    cloud.points[0].y = v[1];
    cloud.points[0].z = v[2];
    // organised as posx  posy posz
    cloud.points[1].x = v[3];
    cloud.points[1].y = v[4];
    cloud.points[1].z = v[5];

    track_pub.publish(cloud);

    ros::spinOnce();
    loop_rate.sleep();
  } // end loop

  return 0;
} // end main
