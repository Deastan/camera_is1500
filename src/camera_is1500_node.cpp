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
// #include <duration.h>

// messages
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"

// include library
#include "interface.h"



int main(int argc, char **argv)
{
  // std::cout << "Ich bin da" << std::endl;
  // system("/home/jonathan/ros_catkin_kyb/src/camera_is1500/src/scriptSHub.bash");
  // ros::init(argc, argv, "interface_sf_ros_node");
  // ROS_INFO("Salut c'est Joli!");
  // ros::start();

  std::vector<float> v;
  // v.push_back(0);
  // v.push_back(0);
  // v.push_back(0);
  // v.push_back(0);
  // v.push_back(0);
  // v.push_back(0);
  overInit();
  ros::init(argc, argv, "interface_sf_ros_node");
  // ros::Rate loop_rate(10);
  // ros::NodeHandle nh;
  // ros::Publisher chatter_pub = nh.advertise<std_msgs::Int32MultiArray>("msgs", 1000);
  // ros::Rate loop_rate(10);
  // std::string alpha = "";

  // ROS_INFO("Try something");
  // std::cout << "Start the program" << std::endl;

  // overInit();
  // v = overGetData();
  // std::cout << v[0] << " " << v[1] << " " << v[2] << " "
  //   << v[3] << " " << v[4] << " " << v[5] << std::endl;
  //   overClose();

  // int count = 0;
  // while(ros::ok())
  // {
  //
  //   v = overGetData();
  //   // std::cout << count << v[0] << std::endl;
  //   // alpha = std::to_string(1+2+4+7+14);
  //   std::cout << count << ": " << v[0] << " " << v[1] << " " << v[2] << " "
  //   << v[3] << " " << v[4] << " " << v[5] << std::endl;
  //   // ROS_INFO("alpha empty");
  //   count++;
  // }
  // ROS_INFO("Empty");
  // overClose();
  // v = overGetData();
  // std::cout << v[0] << " " << v[1] << " " << v[2] << " "
  //   << v[3] << " " << v[4] << " " << v[5] << std::endl;
// ros::Duration duration(1./5.);
while(ros::ok())
{
  v = overGetData();
  std::cout << v[0] << " " << v[1] << " " << v[2] << " "
  << v[3] << " " << v[4] << " " << v[5] << std::endl;

  // std_msgs::Int32MultiArray msgs;
	// 	//Clear array
	// msgs.data.clear();
	// for (int i = 0; i < 6; i++)
	// {
	// 	msgs.data.push_back(v[i]);
	// }
	// //Publish array
	// chatter_pub.publish(msgs);
	//Let the world know
  ROS_INFO("I published something!");
  // duration.sleep();
  // ros::spinOnce();
  // sleep(2);
}
// while(true)
// {
//   v = overGetData();
//   std::cout << v[0] << " " << v[1] << " " << v[2] << " "
//   << v[3] << " " << v[4] << " " << v[5] << std::endl;
//   duration.sleep();
// }
//
//   // ROS_INFO("End of try");
//   // int count = 0;
//   // while (ros::ok())
//   // {
//   //   std::cout << "hello world " << count << std::endl;
//   //   // test if all work
//   //   ROS_INFO("Salut c'est Joli!");
//   //   //chatter_pub.publish(msg);
//   //   ros::spinOnce();
//   //   loop_rate.sleep();
//   //   ++count;
//   // }
//   //
//   // ROS_INFO("End of loop");
//   // ros::spin();
//   // return 0;
//
//
//   ros::spinOnce();
return 0;
} // end main
