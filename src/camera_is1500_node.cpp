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
// #include <nav_msgs/Odometry.msg.h>
// #include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// include library
#include "interface.h"
#include <cmath>
// #include <Ei gen/Dense>

// Convert radians to degrees
#define RAD2DEG(rad) ((rad) * 180.0 / 3.1415927)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interface_sf_ros_node");
  ros::start();

  // Variable init :
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  float h = 0.83;
  float l = 0.75; // [m] in meter
  float x_c = 0, y_c = 0, z_c = 0; // position measured in the camera_frame
  float x_b = 0, y_b = 0, z_b = 0; // position measured by the camera in bodey_frame (base_link)
  float alpha = 0/180*3.14155927; // angle of the camera with its vertical in radians

  // Camera to base_link
  // Matrix<float, 4, 4> transf_baseToCamera;
  // transf_baseToCamera << 1, 0, 0, l,
  //    0, 1, 0, 0,
  //    0, 0, 1, b,
  //    0, 0, 0, 1;


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
      std::cout << "Error: Failed to open sfAccess : Could be an issue in camera_is1500_node.cpp or in the interface.cpp (library) "  << std::endl;
      return 0;
  }


  std::vector<float> v;
  // float x_before = 0;
  // float vel_x = 0;
  while(nh.ok())
  {
    // float dt = (current_time - last_time).toSec();

    // v is a table of float with the data of the IMU of the camera
    // organised as roll   pitch    yaw   posx   posy   posz
    v = overGetData();

    // // Screen data
    // std::cout << v[0] << " " << v[1] << " " << v[2] << " "
    // << v[3] << " " << v[4] << " " << v[5] << std::endl;

    // vel_x = (v[3] - x_before)/dt;

    // transformation in body reference
    // alpha in radians
    // x_c = sin(alpha)*v[3] - cos(alpha)*v[5];
    // y_c = v[4];
    // z_c = cos(alpha)*v[3] + sin(alpha)*v[5];
    //
    // // x_b = x_c - l;
    // // y_b = y_c;
    // // z_b = z_c - h;
    //
    // x_b = x_c*cos(alpha) + y_c*sin(alpha ) - l*cos(alpha);
    // y_b = - x_c*sin(alpha) + y_c*cos(alpha) +l*sin(alpha);
    // z_b = z_c - h;

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = v[3];
    odom.pose.pose.position.y = v[4];
    odom.pose.pose.position.z = v[5];

    // odom.pose.pose.position.x = x_b;//v[3];
    // odom.pose.pose.position.y = y_b;//v[4];
    // odom.pose.pose.position.z = z_b;//v[5];
    // ODOM ARE USED in DEGREES  !!!
    odom.pose.pose.orientation.x = v[0];
    odom.pose.pose.orientation.y = v[1];
    odom.pose.pose.orientation.z = v[2];
    odom.pose.pose.orientation.w = 1;

    // //set the velocity
    // odom.child_frame_id = "base_link";
    // odom.twist.twist.linear.x = vel_x;
    // odom.twist.twist.linear.y = v[1];
    // odom.twist.twist.angular.z = v[1];

    //publish the message
    track_pub.publish(odom);

    last_time = current_time;

    ros::spinOnce();
    loop_rate.sleep();
  } // end loop

  return 0;
} // end main
