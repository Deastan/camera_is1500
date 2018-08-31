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
// #include "interface.h"

// test for memory:
// #include <cstdint>
// #include <sstream>
#include "libsfaccess.h"

// Convert radians to degrees
#define RAD2DEG(rad) ((rad) * 180.0 / 3.1415927)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interface_sf_ros_node");



  // init. publisher
  ros::NodeHandle nh;
  ros::Publisher track_pub = nh.advertise<sensor_msgs::PointCloud>("track_camera", 1000);
  // declaration of the variables

  // overInit();

  ros::Rate loop_rate(10);
  SfAccess sfa;
  sfa.open();
  std::vector<float> v;
  while(nh.ok())
  {
    SfAccess::TrkData trkData;
    bool trkValid = false;
    sfa.getTrackingDataLatest(&trkValid, &trkData);

    // v is a table of float with the data of the IMU of the camera
    // organised as roll  pitch    yaw   posx   posy   posz
    std::vector<float> v;
    // std::cout << "interface.cpp : " << RAD2DEG(trkData.trkRot[0]) << " "
    // << RAD2DEG(trkData.trkRot[1]) << " "
    // << RAD2DEG(trkData.trkRot[2]) << " "
    // << trkData.trkPos[0] << " "
    // << trkData.trkPos[1] << " "
    // << trkData.trkPos[2] << std::endl;
    v.push_back(RAD2DEG(trkData.trkRot[0]));
    v.push_back(RAD2DEG(trkData.trkRot[1]));
    v.push_back(RAD2DEG(trkData.trkRot[2]));
    v.push_back(trkData.trkPos[0]);
    v.push_back(trkData.trkPos[1]);
    v.push_back(trkData.trkPos[2]);
    std::cout << "interface.cpp5 : " << v[0] << " " << v[1] << " " << v[2] << " "
      << v[3] << " " << v[4] << " " << v[5] << std::endl;
    // overInit();
    // v is a table of float with the data of the IMU of the camera
    // organised as roll   pitch    yaw   posx   posy   posz
    // v = overGetData();
    // std::cout << v[0] << " " << v[1] << " " << v[2] << " "
    // << v[3] << " " << v[4] << " " << v[5] << std::endl;

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
    // for(int i = 0;i<6;i++)
    // {
    //   v[i]++;
    // }

    track_pub.publish(cloud);

    // const float* p1 = &v[0];
    // const float* p2 = &v[1];
    // const float* p3 = &v[2];
    // const float* p4 = &v[3];
    // const float* p5 = &v[4];
    // const float* p6 = &v[5];

    // // Screen the variables and memory
    // std::cout << "***************start*****************" << '\n';
    // std::cout << "the int at address " << p1 << " is " << *p1 << '\n';
    // std::cout << "the int at address " << p2 << " is " << *p2 << '\n';
    // std::cout << "the int at address " << p3 << " is " << *p3 << '\n';
    // std::cout << "the int at address " << p4 << " is " << *p4 << '\n';
    // std::cout << "the int at address " << p5 << " is " << *p5 << '\n';
    // std::cout << "the int at address " << p6 << " is " << *p6 << '\n';
    // std::cout << "****************end******************" << '\n';

    ros::spinOnce();
    loop_rate.sleep();
  } // end loop

  return 0;
} // end main
