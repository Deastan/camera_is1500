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


double last_x = 0;
double last_y = 0;
double last_yaw = 0;
double last_vx = 0;
double last_vyaw = 0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interface_sfHub_ros_node");
  ros::start();
  ros::Time last_time;

  // Variable init :
  // Distance between
  float l = 0.835; // [m] in meter

  // init. publisher
  ros::NodeHandle nh;
  // ros::Publisher track_pub = nh.advertise<sensor_msgs::PointCloud>("track_camera", 1000);
  ros::Publisher track_pub = nh.advertise<nav_msgs::Odometry>("position_camera_is1500", 1000);
  ros::Publisher odom_track_pub = nh.advertise<nav_msgs::Odometry>("base_link_odom_camera_is1500", 1000);
  // ros::Publisher odom_track_tf_pub = nh.advertise<nav_msgs::Odometry>("base_link_odom_camera_is1500_TF", 1000);

  // Tf2
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);


  ros::Rate loop_rate(10);
  // Open buffer to take data from the camera,
  // Read sfaccess.ini file and open tracker interface
  if (!overInit())
  {
      // std::cout << "Error: Failed to open sfAccess : Could be an issue in camera_is1500_node.cpp or in the interface.cpp (library) "  << std::endl;
      ROS_WARN("Error: Failed to open sfAccess : Could be an issue in camera_is1500_node.cpp or in the interface.cpp (library)");
      return 0;
  }
  ROS_INFO_STREAM("Camera is connected");

  std::vector<float> v;
  ROS_INFO_STREAM("Geting data from the camera");
  while(nh.ok())
  {
    v = overGetData();
    // velocity
    float curr_x = v[3];
  	float curr_y = v[4];
    ros::Time current_time = ros::Time::now();
  	float dx = (curr_x - last_x);
  	float dy = (curr_y - last_y);
  	double dt = (current_time - last_time).toSec();
  	double vel_x = dx/dt; //- last_x)/dt;
    double vel_y = dy/dt;
    double vel_yaw = (v[2] - last_yaw/dt);

    // float dt = (current_time - last_time).toSec();
    geometry_msgs::TransformStamped transformStamped;

    // v is a table of float with the data of the IMU of the camera
    // organised as roll   pitch    yaw   posx   posy   posz
    // vel_x vel_y vel_z omega_1 omega_2 omega_3

    // std::cout << v[3] << ", " << v[4] << ", " << v[5] << ", " <<
    //   v[0] << ", " << v[1] << ", " << v[2] << ", " << std::endl;

    geometry_msgs::PointStamped base_camera_position;
    base_camera_position = geometry_msgs::PointStamped();//TODO unused
    base_camera_position.header.frame_id = "base_camera";//TODO unused
    base_camera_position.header.stamp = current_time;//TODO unused
    base_camera_position.point.x = v[3];//TODO unused
    base_camera_position.point.y = v[4];//TODO unused
    base_camera_position.point.z = v[5];//TODO unused//TODO unused

    // try{
    // // lookupTransform(const std::string &  	target_frame,
    // // 	const std::string &  	source_frame,
    // // 	const ros::Time &  	time ) 		const)
    // //     Parameters:
    // //     target_frame	The frame to which data should be transformed
    // //     source_frame	The frame where the data originated time
    // //     The time at which the value of the transform is desired. (0 will get the latest)
    // //     Returns:
    // //     The transform between the frames
    //   transformStamped = tfBuffer.lookupTransform("base_link", "odom_camera",
    //                            ros::Time(0));//TODO unused
    // }
    // catch (tf2::TransformException &ex) {
    //   ROS_WARN("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    //   continue;
    // }
    //
    // tf2::doTransform(base_camera_position, base_camera_position, transformStamped);//TODO unused

    std::cout << curr_x << ", " << curr_y << ", " << v[2] << ", " <<
      vel_x << ", " << vel_y << ", " << vel_yaw << ", "  << std::endl;

    // std::cout << v[3] << ", " << v[4] << ", " << v[5] << ", " <<
    //   v[0] << ", " << v[1] << ", " << v[2] << ", " <<
    //     v[6] << ", " << v[7] << ", " << v[8] << ", " <<
    //     v[9] << ", " << v[10] << ", " << v[11] << ", " << std::endl;
    // Position of the camera in the reference of the camera
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom_camera";
    // odom.child_frame_id = "base_camera";
    //set the position
    odom.pose.pose.position.x = v[3];
    odom.pose.pose.position.y = v[4];
    odom.pose.pose.position.z = v[5];

    // ODOM ARE USED in Quaternion  !!!
    tf::Quaternion quat = tf::createQuaternionFromRPY(
                          DEGTORAD(v[0]), DEGTORAD(v[1]), DEGTORAD(v[2]));
    odom.pose.pose.orientation.x = quat[0];
    odom.pose.pose.orientation.y = quat[1];
    odom.pose.pose.orientation.z = quat[2];
    odom.pose.pose.orientation.w = quat[3];
    // //set the velocity
    odom.child_frame_id = "base_camera";
    odom.twist.twist.linear.x = vel_x;
    odom.twist.twist.linear.y = vel_y;
    odom.twist.twist.angular.z = vel_yaw;

    // Transform the position of the camera to base_link manually
    // Create this quaternion from roll/pitch/yaw (in radians)
    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, DEGTORAD(v[2]));


    nav_msgs::Odometry base_link_frame_odom_from_camera;
    base_link_frame_odom_from_camera.header.stamp = current_time;
    base_link_frame_odom_from_camera.header.frame_id = "odom";
    //set the position
    base_link_frame_odom_from_camera.pose.pose.position.x = v[3]-cos(DEGTORAD(v[2]))*l;
    base_link_frame_odom_from_camera.pose.pose.position.y = v[4]-sin(DEGTORAD(v[2]))*l;
    base_link_frame_odom_from_camera.pose.pose.position.z = 0;

    // ODOM ARE USED in quat  !!!
    base_link_frame_odom_from_camera.pose.pose.orientation.x = q[0];
    base_link_frame_odom_from_camera.pose.pose.orientation.y = q[1];
    base_link_frame_odom_from_camera.pose.pose.orientation.z = q[2];//tf::createQuaternionMsgFromYaw(sin(base_camera_position.point.y/l));
    base_link_frame_odom_from_camera.pose.pose.orientation.w = q[3];
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = pow(pow(vel_x, 2.0) + pow(vel_y, 2.0), 0.5);
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = vel_yaw;

    // // Transform the camera frame to base_link with tf
    // nav_msgs::Odometry camera_to_base_link_with_tf;
    // camera_to_base_link_with_tf.header.stamp = current_time;
    // camera_to_base_link_with_tf.header.frame_id = "base_link";
    // //set the position
    // camera_to_base_link_with_tf.pose.pose.position.x = base_camera_position.point.x;
    // camera_to_base_link_with_tf.pose.pose.position.y = base_camera_position.point.y;
    // camera_to_base_link_with_tf.pose.pose.position.z = 0;
    //
    // // ODOM ARE USED in quat  !!!
    // camera_to_base_link_with_tf.pose.pose.orientation.x = 0;
    // camera_to_base_link_with_tf.pose.pose.orientation.y = 0;
    // camera_to_base_link_with_tf.pose.pose.orientation.z = q[2];
    // camera_to_base_link_with_tf.pose.pose.orientation.w = 1;

    //publish the message
    track_pub.publish(odom);
    odom_track_pub.publish(base_link_frame_odom_from_camera);
    // odom_track_tf_pub.publish(camera_to_base_link_with_tf);
    last_time = ros::Time::now();
  	last_x = curr_x;
  	last_y = curr_y;
    last_yaw = v[2];

    ros::spinOnce();
    loop_rate.sleep();
  } // end loop

  if (!overClose())
  {
      // std::cout << "Error: Failed to open sfAccess : Could be an issue in camera_is1500_node.cpp or in the interface.cpp (library) "  << std::endl;
      ROS_WARN("Failed to close sfAccess");
      return 0;
  }else
  {
    ROS_INFO_STREAM("sfAccess is closed");
  }
  ROS_INFO_STREAM("End of the node : camera_is1500_node");
  // std::cout << "End of the node : camera_is1500_node" << std::endl;
  return 0;
} // end main
