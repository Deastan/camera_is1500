/*
* Camera is1500 data acquisition which publish through ROOS
* Jonathan Burkhard, Kyburz 2018
* Documentations :
* https://docs.google.com/document/d/1PqNxtafhbULaRYPs3b4RwIS-OikPkcjWYqzIcSp_Swk/edit?usp=sharing
*/

#include "ros/ros.h"
#include <sstream>
#include <ros/console.h>
#include <boost/bind.hpp>
#include <stdio.h>
#include <ros/callback_queue.h>
// include libraries
#include <string>
#include "interface.h"
#include <cmath>

// messages
#include <nav_msgs/Odometry.h>

// tf1
 #include <tf/tf.h>
 #include <tf/transform_broadcaster.h>

// tf2 transformation
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Convert radians / degrees
#define RAD2DEG(rad) ((rad) * 180.0 / 3.1415927)
#define DEGTORAD(deg) ((deg) / 180.0 * 3.1415927)

double test; // Only here fore test...

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interface_sfHub_ros");
  ros::start();
  ros::Time last_time;
  tf2_ros::TransformBroadcaster br;

  // Variables initialization :
  double last_x = 0;
  double last_y = 0;
  double last_yaw = 0;
  double last_vx = 0;
  double last_vyaw = 0;


  // init. publisher
  ros::NodeHandle nh;//if private param can put that after nh like nh("~");
  ros::Publisher track_pub = nh.advertise<nav_msgs::Odometry>("position_camera_is1500", 1000);
  ros::Publisher odom_track_pub = nh.advertise<nav_msgs::Odometry>("base_link_odom_camera_is1500", 1000);

  double originX;
  if(!nh.getParam("/camera_is1500_node/originX", originX))
  {
    ROS_ERROR("Could not find topic parameter : /camera_is1500_node/originX");
  }
  double originY;
  if(!nh.getParam("/camera_is1500_node/originY", originY))
  {
    ROS_ERROR("Could not find topic parameter : /camera_is1500_node/originY");
  }
  double rotationAngleCamToUTM;
  if(!nh.getParam("/camera_is1500_node/rotationAngleCamToUTM", rotationAngleCamToUTM))
  {
    ROS_ERROR("Could not find topic parameter : /camera_is1500_node/rotationAngleCamToUTM");
  }
  double l = 0.835; // [m] in meter // Distance between center of the robot and the camera
  if(!nh.getParam("/camera_is1500_node/l_camera_x", l))
  {
    ROS_ERROR("Could not find topic parameter : /camera_is1500_node/l_camera_x");
  }

  // Tf2
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate loop_rate(10);

  // Open buffer to take data from the camera,
  // Read sfaccess.ini file and open tracker interface
  if (!overInit())
  {
      ROS_WARN("Error: Failed to open sfAccess : Could be an issue in camera_is1500_node.cpp or in the interface.cpp (library)");
      return 0;
  }else
  {
    ROS_INFO_STREAM("Camera is connected");
  }

  std::vector<float> v;
  ROS_INFO_STREAM("Geting data from the camera");
  while(nh.ok())
  {
    // v is a table of float with the data of the IMU of the camera
    // organised as roll   pitch    yaw   posx   posy   posz
    v = overGetData();
    // std::cout << v[3] << ", " << v[4] << ", " << v[5] << ", " <<
    //   v[0] << ", " << v[1] << ", " << v[2] << ", " << std::endl;
    
    // Calculation of the velocity
    float curr_x = v[3];
  	float curr_y = v[4];
    ros::Time current_time = ros::Time::now();
  	float dx = (curr_x - last_x);
  	float dy = (curr_y - last_y);
  	double dt = (current_time - last_time).toSec();
  	double vel_x = dx/dt; //- last_x)/dt;
    double vel_y = dy/dt;
    double vel_yaw = (v[2] - last_yaw/dt);

    // Publish the transforms over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom_refOfcam";
    odom_trans.child_frame_id = "base_link_refOfcam";
    odom_trans.transform.translation.x = curr_x-cos(DEGTORAD(v[2]))*l;
    odom_trans.transform.translation.y = curr_y-sin(DEGTORAD(v[2]))*l;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(DEGTORAD(v[2]));
    br.sendTransform(odom_trans);

    // // replace odom if needed
    // geometry_msgs::TransformStamped odom_trans;
    // odom_trans.header.stamp = current_time;
    // odom_trans.header.frame_id = "odom";
    // odom_trans.child_frame_id = "base_link";
    // odom_trans.transform.translation.x = curr_x-cos(DEGTORAD(v[2]))*l;
    // odom_trans.transform.translation.y = curr_y-sin(DEGTORAD(v[2]))*l;
    // odom_trans.transform.translation.z = 0.0;
    // odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(DEGTORAD(v[2]));
    // br.sendTransform(odom_trans);

    double centerRobotPoseX = curr_x-cos(DEGTORAD(v[2]))*l;
    double centerRobotPoseY = curr_y-sin(DEGTORAD(v[2]))*l;
    // In GPS coordinates
    geometry_msgs::TransformStamped odom_trans2;
    odom_trans2.header.stamp = current_time;
    odom_trans2.header.frame_id = "odom";
    odom_trans2.child_frame_id = "base_link";
    odom_trans2.transform.translation.x = centerRobotPoseX * cos(rotationAngleCamToUTM) + centerRobotPoseY * cos(1.57079632679 - rotationAngleCamToUTM) - 65.88;
    odom_trans2.transform.translation.y = centerRobotPoseX * sin(rotationAngleCamToUTM) + centerRobotPoseY * sin(1.57079632679 - rotationAngleCamToUTM) - 55.38;
    odom_trans2.transform.translation.z = 0.0;
    odom_trans2.transform.rotation = tf::createQuaternionMsgFromYaw(DEGTORAD(v[2]) + rotationAngleCamToUTM);
    br.sendTransform(odom_trans2);

    // Position of the camera in the reference of the camera
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "pose_Camera_refOfcam";
    // Set the position
    odom.pose.pose.position.x = v[3];
    odom.pose.pose.position.y = v[4];
    odom.pose.pose.position.z = v[5];

    tf::Quaternion quat = tf::createQuaternionFromRPY(
                          DEGTORAD(v[0]), DEGTORAD(v[1]), DEGTORAD(v[2]));
    // Set the attitude
    odom.pose.pose.orientation.x = quat[0];
    odom.pose.pose.orientation.y = quat[1];
    odom.pose.pose.orientation.z = quat[2];
    odom.pose.pose.orientation.w = quat[3];
    // Set the velocity
    odom.twist.twist.linear.x = vel_x;
    odom.twist.twist.linear.y = vel_y;
    odom.twist.twist.angular.z = vel_yaw;

    // Transform the position of the camera to base_link manually
    // Create this quaternion from roll/pitch/yaw (in radians)
    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, DEGTORAD(v[2]));

    nav_msgs::Odometry base_link_frame_odom_from_camera;
    base_link_frame_odom_from_camera.header.stamp = current_time;
    base_link_frame_odom_from_camera.header.frame_id = "odom_refOfcam";
    base_link_frame_odom_from_camera.child_frame_id = "base_link_refOfcam";
    //set the position
    base_link_frame_odom_from_camera.pose.pose.position.x = v[3]-cos(DEGTORAD(v[2]))*l;
    base_link_frame_odom_from_camera.pose.pose.position.y = v[4]-sin(DEGTORAD(v[2]))*l;
    base_link_frame_odom_from_camera.pose.pose.position.z = 0;

    // ODOM ARE USED in quat  !!!
    base_link_frame_odom_from_camera.pose.pose.orientation.x = q[0];
    base_link_frame_odom_from_camera.pose.pose.orientation.y = q[1];
    base_link_frame_odom_from_camera.pose.pose.orientation.z = q[2];//tf::createQuaternionMsgFromYaw(sin(base_camera_position.point.y/l));
    base_link_frame_odom_from_camera.pose.pose.orientation.w = q[3];

    // set the velocity
    base_link_frame_odom_from_camera.child_frame_id = "base_link_refOfcam";
    base_link_frame_odom_from_camera.twist.twist.linear.x = pow(pow(vel_x, 2.0) + pow(vel_y, 2.0), 0.5);
    base_link_frame_odom_from_camera.twist.twist.linear.y = 0;
    base_link_frame_odom_from_camera.twist.twist.angular.z = vel_yaw;

    // Publish the message
    track_pub.publish(odom);
    odom_track_pub.publish(base_link_frame_odom_from_camera);

    // Set for next loop
    last_time = ros::Time::now();
  	last_x = curr_x;
  	last_y = curr_y;
    last_yaw = v[2];

    ros::spinOnce();
    loop_rate.sleep();
  } // end ros-loop

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
  return 0;
} // end main
