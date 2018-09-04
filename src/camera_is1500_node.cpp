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
#include <nav_msgs/Odometry.msg.h>

// include library
#include "interface.h"

// Convert radians to degrees
#define RAD2DEG(rad) ((rad) * 180.0 / 3.1415927)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interface_sf_ros_node");
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  // init. publisher
  ros::NodeHandle nh;
  // ros::Publisher track_pub = nh.advertise<sensor_msgs::PointCloud>("track_camera", 1000);
  ros::Publisher track_pub = nh.advertise<nav_msgs::Odometry>("odom_camera_is1500", 1000);

  ros::Rate loop_rate(10);
  //SfAccess sfa;

  // Open buffer to take data from the camera,
  // Read sfaccess.ini file and open tracker interface
  if (!overInit())
  {
      std::cout << "Error: Failed to open sfAccess : Could be an issue in
        camera_is1500_node.cpp or in the interface.cpp (library) "  << std::endl;
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
    // sensor_msgs::PointCloud cloud;
    // nav_msgs::Odometry msg;
    //
    // // cloud.header.stamp = ros::Time::now();
    // // cloud.header.frame_id = "camera_is1500";
    // //
    // // cloud.points.resize(2);
    // // // organised as roll  pitch yaw
    // // cloud.points[0].x = v[0];
    // // cloud.points[0].y = v[1];
    // // cloud.points[0].z = v[2];
    // // // organised as posx  posy posz
    // // cloud.points[1].x = v[3];
    // // cloud.points[1].y = v[4];
    // // cloud.points[1].z = v[5];
    //
    // track_pub.publish(cloud);


    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = v[3];
    odom.pose.pose.position.y = v[4];
    odom.pose.pose.position.z = v[5];
    // ODOM ARE IN DEGREES  USED !!!
    odom.pose.pose.orientation.x = v[0];
    odom.pose.pose.orientation.y = v[1];
    odom.pose.pose.orientation.z = v[2];
    odom.pose.pose.orientation.w = 1;


    // //set the velocity
    // odom.child_frame_id = "base_link";
    // odom.twist.twist.linear.x = v[0];
    // odom.twist.twist.linear.y = v[1];
    // odom.twist.twist.angular.z = v[1];

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;

    ros::spinOnce();
    loop_rate.sleep();
  } // end loop

  return 0;
} // end main
