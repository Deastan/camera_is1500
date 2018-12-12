/*
* Camera is1500 data acquisition which publish through ROOS
* Jonathan Burkhard, Kyburz 2018
* Documentations :
* https://docs.google.com/document/d/1PqNxtafhbULaRYPs3b4RwIS-OikPkcjWYqzIcSp_Swk/edit?usp=sharing
*/

// TODO:
// - Write a file config for placement of the map file and sfHub
// - Write a script for building interface library with right .ini setting
// - Clean the code with function, private variables etc...
// - Remove the rosparam to change map because it can save a lot of time
//    inside the loop

// Include
#include <camera_is1500_node.h>

#include <camera_is1500/GetPointsFromMetric.h>

#include <XmlRpcValue.h>
#include "ros/console.h"
#include "ros/static_assert.h"

// Variables initialization :
double frequency = 100; // Hz
// velocity
double last_x = 0;
double last_y = 0;
double last_yaw = 0;
double last_vx = 0;
double last_vyaw = 0;
// Variances and Covariances
int size_var = 60;
std::vector<double> x_vec(size_var, 0.0);
std::vector<double> y_vec(size_var, 0.0);
// double x_vec[size_var] = {};
double TWIST_COVAR [6][6] = {{0.0001, 0, 0, 0, 0, 0},
               {0, 0.0001, 0, 0, 0, 0},
               {0, 0, 0.0001, 0, 0, 0},
               {0, 0, 0, 0.0001, 0, 0},
               {0, 0, 0, 0, 0.0001, 0},
               {0, 0, 0, 0, 0, 0.0001}};
double l_x; // [m] in meter // Distance between center of the robot and the camera
double l_y; // [m] in meter // Distance between center of the robot and the camera
double angleCameraConfig;
double l = sqrt(pow(l_x,2) + pow(l_y, 2)); // distance btw camera and robot center
int mapNumber = 0;
int lastMapNumber = -1;
std::vector<string> tableMapPaths; // GPS
std::vector<float> v;
// metric
double err_dist = 0;
double predict_x = 0;
double predict_y = 0;
double err_predict = 0;
double increment_time = 1/frequency; // in seconde
double err_angle = 0;
double cost_function = 0;
double threshold = 1.65;
double alpha_1 = 1.5; //err_dist
double alpha_2 = 1.5; //err_predict
double alpha_3 = 0.5; //err_angle
bool publish_tf_bool = 1;
ros::Publisher *metric_pub;
//******************************************************************************
//  MAIN
//******************************************************************************
int main(int argc, char **argv)
{
  // init ros vairables
  ros::init(argc, argv, "interface_sfHub_ros");
  ros::start();
  ros::Time last_time;
  tf2_ros::TransformBroadcaster br;

  // init. publisher
  ros::NodeHandle nh;//if private param can put that after nh like nh("~");
  ros::Publisher track_pub = nh.advertise<nav_msgs::Odometry>("position_camera_is1500", 1);//10000 to 1
  ros::Publisher odom_track_pub = nh.advertise<nav_msgs::Odometry>("base_link_odom_camera_is1500", 1); //1000 to 1
  ros::Rate loop_rate(frequency);
  metric_pub =  new ros::Publisher(nh.advertise<camera_is1500::GetPointsFromMetric>("pointsFromMetric", 10)); //1000 to 1
  ros::Subscriber sub_for_metric = nh.subscribe("base_link_odom_camera_is1500", 1, metricCamera);


  init(nh);

  openSfHub();
  ROS_INFO_STREAM("Geting data from the camera");
//******************************************************************************
//  Loop : Get data from the camera and publish it
//******************************************************************************
  while(nh.ok())
  {
    // load map
    changeMap(nh, mapNumber, lastMapNumber, tableMapPaths);
    publish_position(nh, track_pub, odom_track_pub, last_time, br);

    ros::spinOnce();
    loop_rate.sleep();


    // Get parameters
  } // end ros-loop

  closeSfHub();
  ROS_INFO_STREAM("End of the node : camera_is1500_node");
  return 0;
} // end main

//******************************************************************************
//  Definition of functions
//******************************************************************************


// Open SfHub to get the data from the camera in another terminal
void openSfHub()
{
  // Open buffer to take data from the camera,
  // Read sfaccess.ini file and open tracker interface
  if (!overInit())
  {
      ROS_WARN("Error: Failed to open sfAccess : Could be an issue in camera_is1500_node.cpp or in the interface.cpp (library)");
  }else
  {
    ROS_INFO_STREAM("Camera is connected");
  }
}
void closeSfHub()
{
  if (!overClose())
  {
      ROS_WARN("Failed to close sfAccess");
  }else
  {
    ROS_INFO_STREAM("sfAccess is closed");
  }
}

// Calculate covariance of a vector of double
// INPUT: vector of double
// OUTPUT: double variable of covairance
double compute_variance(std::vector<double> v)
{
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  double mean = sum / v.size();

  std::vector<double> diff(v.size());
  std::transform(v.begin(), v.end(), diff.begin(),
                 std::bind2nd(std::minus<double>(), mean));

  // for standard deviation
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  // double stdev = std::sqrt(sq_sum / v.size());
  return (sq_sum / v.size());
}

// Initializate variables from cfg file
void init(ros::NodeHandle nh)
{
  if(!nh.getParam("/camera_is1500_node/l_camera_x", l_x))
  {
    ROS_ERROR("Could not find topic parameter : /camera_is1500_node/l_camera_x");
    l_x = 0.0; // set default value
  }

  if(!nh.getParam("/camera_is1500_node/l_camera_y", l_y))
  {
    ROS_ERROR("Could not find topic parameter : /camera_is1500_node/l_camera_y");
    l_y = 0.350; // set default value
  }

  l = sqrt(pow(l_x,2) + pow(l_y, 2)); // distance btw camera and robot center

  if(!nh.getParam("/camera_is1500_node/mapNumber", mapNumber))
  {
    ROS_ERROR("Could not find topic parameter : /camera_is1500_node/mapNumber");
  }
  if(!nh.getParam("/camera_is1500_node/angleCameraConfig", angleCameraConfig))
  {
    ROS_ERROR("Could not find topic parameter : /camera_is1500_node/mapNumber");
  }

  if(!nh.getParam("/camera_is1500_node/tableMapPaths", tableMapPaths))
  {
    ROS_ERROR("Could not find topic parameter : /camera_is1500_node/tableMapPaths");
  }
  std::cout << "Loaded maps: " << " " << std::endl;
  for(int i = 0;i<tableMapPaths.size();i++)
  {
    std::cout << i+1 << ": " << tableMapPaths[i] << " " << std::endl;
  }

}

// Change the map in sfHub in closing sfHub and re-run sfHub with new loaded map
// 0 = default
// 1 = hangar
// 2 = ...
// TODO write it for real system
// TODO perhaps it create a suscriber to a message receiving a command
//      from the terminal new map and not from ros param
void changeMap(ros::NodeHandle nh, int &numberMap, int &lastMapNumber,
  std::vector<string> tableMapPathsArg)
{
  nh.getParam("/camera_is1500_node/mapNumber", numberMap);
  if(numberMap < tableMapPathsArg.size() and numberMap >= 0 and numberMap != lastMapNumber)
  {
    system("gnome-terminal -x sh -c 'pkill sfHub'");
    ROS_INFO("Map is configured");
    std::ifstream  src(tableMapPathsArg[numberMap].c_str(), std::ios::binary);
    std::cout << "Map memory: " << src << ", path is: " << tableMapPathsArg[numberMap] << std::endl;
    std::ofstream  dst("/home/ew/Documents/wrapperCameraIS-1500/IS-1500_Software/Linux/sfHub/S1/environmentPSEs.cfg",   std::ios::binary);
    dst << src.rdbuf();
    system("gnome-terminal -x sh -c 'cd && cd /home/ew/Documents/wrapperCameraIS-1500/IS-1500_Software/Linux/sfHub/ && ./sfHub'");
    // system("gnome-terminal -x sh -c 'cd && cd && cd catkin_ws_kyb && source devel/setup.bash && roscd camera_is1500/ && cd external_software/sfHub/ && ./sfHub'");
    lastMapNumber = numberMap;
    // return true;
    ros::Duration(5.0).sleep(); // sleep for 5 seconds
  } // end if
} // end function

// Publish data of the get position on the ros cloud
void publish_position(ros::NodeHandle nh, ros::Publisher track_pub,
  ros::Publisher odom_track_pub, ros::Time &last_time,
  tf2_ros::TransformBroadcaster br)
{
  // v is a table of float with the data of the IMU of the camera
  // organised as roll   pitch    yaw   posx   posy   posz
  v = overGetData();
  //std::cout << v[3] << ", " << v[4] << ", " << v[5] << ", " <<
  //   v[0] << ", " << v[1] << ", " << v[2] << ", " << std::endl;

  // Calculation of the robot's velocity
  float curr_x = v[3];
  float curr_y = v[4];
  double yaw_cam = v[2]; // in deg
  double yaw;
  if(yaw_cam < 180.0 and yaw_cam>0.0 )
  {
    yaw =  v[2] - angleCameraConfig;
  }else if(yaw_cam>-180.0 and yaw_cam<-90.0)
  {
    yaw = 180.0 + v[2] + angleCameraConfig;
  }else
  {
    yaw = v[2] - angleCameraConfig;
  }

  ros::Time current_time = ros::Time::now();
  float dx = (curr_x - last_x);
  float dy = (curr_y - last_y);
  double dt = (current_time - last_time).toSec();
  double vel_x = dx/dt; //- last_x)/dt;
  double vel_y = dy/dt;
  double vel_yaw = (yaw - last_yaw/dt);

  double centerRobotPoseX = curr_x - cos(DEGTORAD(yaw_cam))*l;
  double centerRobotPoseY = curr_y - sin(DEGTORAD(yaw_cam))*l;

  // Publish the transforms over tf if publish_tf_bool == True
  // TODO :
  //        - switch with GPS
  if(publish_tf_bool)
  {
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = centerRobotPoseX;
    odom_trans.transform.translation.y = centerRobotPoseY;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(DEGTORAD(yaw));
    br.sendTransform(odom_trans);
  }

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
  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, DEGTORAD(yaw));

  nav_msgs::Odometry base_link_frame_odom_from_camera;
  base_link_frame_odom_from_camera.header.stamp = current_time;
  base_link_frame_odom_from_camera.header.frame_id = "odom";
  base_link_frame_odom_from_camera.child_frame_id = "base_link";
  //set the position
  base_link_frame_odom_from_camera.pose.pose.position.x = centerRobotPoseX;
  base_link_frame_odom_from_camera.pose.pose.position.y = centerRobotPoseY;
  base_link_frame_odom_from_camera.pose.pose.position.z = 0;

  // ODOM ARE USED in quat  !!!
  base_link_frame_odom_from_camera.pose.pose.orientation.x = q[0];
  base_link_frame_odom_from_camera.pose.pose.orientation.y = q[1];
  base_link_frame_odom_from_camera.pose.pose.orientation.z = q[2];//tf::createQuaternionMsgFromYaw(sin(base_camera_position.point.y/l));
  base_link_frame_odom_from_camera.pose.pose.orientation.w = q[3];

  // set the velocity
  base_link_frame_odom_from_camera.child_frame_id = "base_link";
  base_link_frame_odom_from_camera.twist.twist.linear.x = pow(pow(vel_x, 2.0) + pow(vel_y, 2.0), 0.5);
  base_link_frame_odom_from_camera.twist.twist.linear.y = 0;
  base_link_frame_odom_from_camera.twist.twist.angular.z = vel_yaw;

  // Insert last 60 position in array of position x, y
  x_vec.insert(x_vec.begin(), base_link_frame_odom_from_camera.pose.pose.position.x);
  y_vec.insert(y_vec.begin(), base_link_frame_odom_from_camera.pose.pose.position.y);
  // remove the oldest line
  x_vec.pop_back();
  y_vec.pop_back();
  // std::cout << "variance of x :" << compute_variance(x_vec) << "variance of y :" << compute_variance(y_vec) << std::endl;

  // Add covariance to message
  base_link_frame_odom_from_camera.pose.covariance[21] = compute_variance(x_vec);
  base_link_frame_odom_from_camera.pose.covariance[28] = compute_variance(y_vec);

  // base_link_frame_odom_from_camera.twist.covariance = TWIST_COVAR;

  // Publish the message
  track_pub.publish(odom);
  odom_track_pub.publish(base_link_frame_odom_from_camera);

  // Set for next loop
  last_time = ros::Time::now();
  last_x = curr_x;
  last_y = curr_y;
  last_yaw = yaw;
}

// Metric : switch/remove the sending of tf from the camera
// TODO
// - add a jump of velocity using the grad DONE
// - add the Yaw measure
// - Optimize the cost function parameters
// - Compare to the an trajectory and not only to the next point...
void metricCamera(const nav_msgs::Odometry::ConstPtr& msg)
{
  double pos_x = msg->pose.pose.position.x;
  double pos_y = msg->pose.pose.position.y;
  // Close to covariance
  // Compare the error between the position actual position with the one before
  err_dist = pow(pow(pos_x - last_x, 2) + pow(pos_y - last_y, 2), 0.5);

  // Error btw where it should be and where it is
  predict_x = (last_x + increment_time * cos(last_yaw) * msg->twist.twist.linear.x);
  predict_y = (last_y + increment_time * sin(last_yaw) * msg->twist.twist.linear.x);
  err_predict = pow((pow(predict_x - pos_x, 2) +
    pow(predict_y - pos_y, 2)), 0.5);
  // Error in btw prediction direction and direction of the robot (normalized)
  err_angle = (abs((atan2(predict_y - last_y,predict_x - last_x) -
    atan2(pos_y - last_y, pos_x - last_x))/(2*M_PI)));

  // Cost vector : sum of the cost
  cost_function = (alpha_1 * err_dist + alpha_2 * err_predict +
    alpha_3 * err_angle);
  // std::cout << "cost_vector: " << cost_function << ", err_dist: " << err_dist <<
  //   "err_predict: " << err_predict << ", err_angle: " << err_angle << std::endl;

  if(cost_function > threshold)
  {
    // std::cout << "Aie aie aie" << std::endl;
    // publish_tf_bool = !publish_tf_bool;
    // test messages
    camera_is1500::GetPointsFromMetric msg_to_pub;
    msg_to_pub.x = pos_x;
    msg_to_pub.y = pos_y;
    msg_to_pub.z = 0.0;
    double roll = msg->pose.pose.orientation.x;
    double pitch = msg->pose.pose.orientation.y;
    double yaw = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;
    yaw = atan2(2.0*((w*yaw+roll*pitch)), 1.0-2.0*(pow(pitch, 2)+pow(yaw,2)));
    msg_to_pub.yaw = yaw;
    msg_to_pub.cost = cost_function;
    msg_to_pub.mapNumber = mapNumber;
    metric_pub->publish(msg_to_pub);
  }
}

// Straiht line until camera metric is good
void deadRecon(ros::NodeHandle nh)
{
  // ros::Subscriber sub_wheel_odom = nh.subscribe("wheel_odom", 1, );
}
