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

// tf1
 #include <tf/tf.h>
// tf2 transformation
#include <tf2_ros/buffer.h>
// #include <tf2/transform_datatypes.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// // #include <tf2_ros/buffer.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/PointStamped.h" //TODO VERIFY THIS
// // #include <buffer.h>
// #include "tf2_ros/message_filter.h"
// #include "message_filters/subscriber.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// // #include <tf2/LinearMath/Quaternion.h>


// Convert radians / degrees
#define RAD2DEG(rad) ((rad) * 180.0 / 3.1415927)
#define DEGTORAD(deg) ((deg) / 180.0 * 3.1415927)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interface_sf_ros_node");
  ros::start();

  // Variable init :
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  // float h = 0.83;
  float l = 0.75; // [m] in meter
  // float x_c = 0, y_c = 0, z_c = 0; // position measured in the camera_frame
  // float x_b = 0, y_b = 0, z_b = 0; // position measured by the camera in bodey_frame (base_link)
  // float alpha = 0/180*3.14155927; // angle of the camera with its vertical in radians

  // init. publisher
  ros::NodeHandle nh;
  // ros::Publisher track_pub = nh.advertise<sensor_msgs::PointCloud>("track_camera", 1000);
  ros::Publisher track_pub = nh.advertise<nav_msgs::Odometry>("odom_camera_is1500", 1000);
  ros::Publisher odom_track_pub = nh.advertise<nav_msgs::Odometry>("base_link_odom_camera_is1500", 1000);

  // Tf2
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);


  ros::Rate loop_rate(10);
  // Open buffer to take data from the camera,
  // Read sfaccess.ini file and open tracker interface
  if (!overInit())
  {
      std::cout << "Error: Failed to open sfAccess : Could be an issue in camera_is1500_node.cpp or in the interface.cpp (library) "  << std::endl;
      return 0;
  }

  std::vector<float> v;
  // std::vector<float> position_base_camera;
  // float x_before = 0;
  // float vel_x = 0;
  while(nh.ok())
  {
    // float dt = (current_time - last_time).toSec();
    geometry_msgs::TransformStamped transformStamped;

    // v is a table of float with the data of the IMU of the camera
    // organised as roll   pitch    yaw   posx   posy   posz
    v = overGetData();

    geometry_msgs::PointStamped base_camera_position;
    base_camera_position = geometry_msgs::PointStamped();
    base_camera_position.header.frame_id = "base_camera";
    base_camera_position.header.stamp = current_time;
    base_camera_position.point.x = v[3];
    base_camera_position.point.y = v[4];
    base_camera_position.point.z = v[5];

    try{
// lookupTransform(const std::string &  	target_frame,
// 	const std::string &  	source_frame,
// 	const ros::Time &  	time ) 		const)
//     Parameters:
//     target_frame	The frame to which data should be transformed
//     source_frame	The frame where the data originated time
//     The time at which the value of the transform is desired. (0 will get the latest)
//     Returns:
//     The transform between the frames
      transformStamped = tfBuffer.lookupTransform("base_link", "base_camera",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    std::cout << "Screen before transfomation : " << base_camera_position.point.x << ", "
    << base_camera_position.point.y << ", " << base_camera_position.point.z << std::endl;

    tf2::doTransform(base_camera_position, base_camera_position, transformStamped);

    std::cout << "Screen after transfomation : " << base_camera_position.point.x << ", "
    << base_camera_position.point.y << ", " << base_camera_position.point.z << std::endl;

    // transformStamped.waitForTransform("/base_camera", "/base_link", current_time, ros::Duration(1.0));
    // transformStamped.getOrigin ();
    // base_camera_position.tr

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom_base_camera";
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

    nav_msgs::Odometry base_camera_odom;
    base_camera_odom.header.stamp = current_time;
    base_camera_odom.header.frame_id = "odom_base_camera";
    //set the position
    base_camera_odom.pose.pose.position.x = base_camera_position.point.x;
    base_camera_odom.pose.pose.position.y = base_camera_position.point.y;
    base_camera_odom.pose.pose.position.z = base_camera_position.point.z;
    // odom.pose.pose.position.x = x_b;//v[3];
    // odom.pose.pose.position.y = y_b;//v[4];
    // odom.pose.pose.position.z = z_b;//v[5];
    // ODOM ARE USED in DEGREES  !!!
    base_camera_odom.pose.pose.orientation.x = 0;
    base_camera_odom.pose.pose.orientation.y = 0;
    base_camera_odom.pose.pose.orientation.z = v[2];//tf::createQuaternionMsgFromYaw(sin(base_camera_position.point.y/l));
    base_camera_odom.pose.pose.orientation.w = 1;
    // //set the velocity
    // odom.child_frame_id = "base_link";
    // odom.twist.twist.linear.x = vel_x;
    // odom.twist.twist.linear.y = v[1];
    // odom.twist.twist.angular.z = v[1];

    //publish the message
    track_pub.publish(odom);
    odom_track_pub.publish(base_camera_odom);

    last_time = current_time;

    ros::spinOnce();
    loop_rate.sleep();
  } // end loop

  return 0;
} // end main
