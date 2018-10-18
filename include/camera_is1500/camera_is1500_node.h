#include <Eigen/Eigen>
#include <boost/bind.hpp>
#include <stdio.h>

// ros
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "overactuated_control/bicopter_controller.h"
#include "rotors_control/common.h"
#include "rotors_control/parameters_ros.h"

// msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

bool changeMap(int numberMap);
