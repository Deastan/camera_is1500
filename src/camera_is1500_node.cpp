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

#include "std_msgs/Int16.h"
#include "sensor_msgs/JointState.h"
// #include "isense.h"
#ifdef UNIX
#include <unistd.h>
#endif

// Convert radians to degrees
#define RAD2DEG(rad) ((rad) * 180.0 / 3.1415927)

int main(int argc, char **argv)
{
//
//   ros::init(argc, argv, "talker");
//
//   ros::NodeHandle n;
//
//   ros::Publisher chatter_pub = n.advertise<std_msgs::Int16>("head_yaw", 1000);
//   ros::Publisher chatter_pub_2 = n.advertise<std_msgs::Int16>("head_accel", 1000);
//
//   ros::Rate loop_rate(10);
//
//   ISD_TRACKER_HANDLE       handle;
//   ISD_TRACKER_INFO_TYPE    tracker;
//   ISD_TRACKING_DATA_TYPE   data;
//
//   handle = ISD_OpenTracker((Hwnd)NULL, 0, FALSE, FALSE );
//
//   ISD_ResetHeading(handle, 1);
//
//   if ( handle > 0 )
//     printf( "\n    Az    El    Rl    X    Y    Z \n" );
//   else
//     printf( "Tracker not found. Press any key to exit" );
//
//   int count = 0;
//   int yaw = 0;
//   int pitch = 0;
//   double yaw_cmd = 0;
//   long counter = 0;
//   while (ros::ok())
//   {
//     counter = counter + 1;
//
//     ISD_GetTrackingData( handle, &data );
//     printf( "%7.2f \n", data.Station[0].AngularVelNavFrame[2]);
//     yaw = (data.Station[0].Euler[0]);
//     yaw_cmd = (float)(yaw) / (float)(60);
//     pitch = data.Station[0].AngularVelNavFrame[2];
//     ISD_GetCommInfo( handle, &tracker );
//
//     std_msgs::Int16 msg;
//     std_msgs::Int16 msg2;
//     sensor_msgs::JointState head_track_msg;
//
//     msg.data = yaw;
//     msg2.data = pitch;
//
//
//
//
//     chatter_pub.publish(msg);
//     chatter_pub_2.publish(msg2);
//
//
//
//     ros::spinOnce();
//     loop_rate.sleep();
//     ++count;
//   }




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
