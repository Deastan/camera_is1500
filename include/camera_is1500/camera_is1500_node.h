#include "ros/ros.h"
#include <sstream>
#include <fstream>

#include <ros/console.h>
#include <boost/bind.hpp>
#include <stdio.h>
#include <ros/callback_queue.h>
// include libraries
#include <string>

// #include "std_msgs/String.h"
#include "interface.h"
// #include <math.h>
#include <cmath>
#include <vector>
#include <numeric>

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

void openSfHub();
void closeSfHub();
void init(ros::NodeHandle nh);
void changeMap(ros::NodeHandle nh, int &numberMap, int &lastMapNumber,
  std::vector<string> tableMapPathsArg);
void publish_position(ros::NodeHandle nh, ros::Publisher track_pub,
  ros::Publisher odom_track_pub, ros::Time &last_time,
  tf2_ros::TransformBroadcaster br);
void metricCamera(const nav_msgs::Odometry::ConstPtr& msg);
void deadRecon(ros::NodeHandle nh);
