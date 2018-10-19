#! /usr/bin/env python

# ET1 controller with camera_is1500
# Jonathan Burkhard, Kyburz 2018
# Documentations : Not exist yet

# TODO
# - Add gain
# - Clean the code with function, private variables etc...

# Basic imports
import sys
import rospy
import time
import tf
import numpy as np
import math
import tf.transformations
from geometry_msgs.msg import Quaternion, Point, Pose, Twist, Vector3
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from math import radians, pow

# from std_msgs.msg import Empty
from std_msgs.msg import String

#*******************************************************************************
#   Usefull functions
#*******************************************************************************
def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

global mot_pub
mot_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1) # 100 to 1
global mot_msg # Contain the msg

global ready_to_go
ready_to_go = rospy.set_param('/waypoint_node/ready_to_go', False)#
global success
success = rospy.set_param('/waypoint_node/success', False)#
global target
target = rospy.set_param('/waypoint_node/target', [1.5,1])
#target = rospy.set_param('/waypoint_node/target', [3.7,-6.0])

def turning_callback(msg):
    # Re-assignement inside func
    global ready_to_go
    global success
    global target
    global mot_msg
    mot_msg = Twist()

    ready_to_go = rospy.get_param('/waypoint_node/ready_to_go')
    success = rospy.get_param('/waypoint_node/success')
    target = rospy.get_param('/waypoint_node/target')

    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    if(ready_to_go == 1 and success == 0): # Success is not used yet...
        u = [np.cos(yaw), np.sin(yaw)] # direction of the robot
        v = [target[0] -robot_x, target[1] - robot_y] # vector btw robot and target
        angle = np.arctan2(v[1], v[0])
        err_angle = angle - yaw # angle(u, v)
        err_x = length(v) # distance btw robot un target
        print("Err_angle : ", err_angle, "Err_x : ", err_x, '\n', "Robot position : (", robot_x, ", ", robot_y, ") ")

        # First change the angle to have the point in front of the robot and
        # after move forward
        if(np.abs(err_angle) > 0.20 and np.abs(err_x) > 0.15): # envi 2*2.9 deg
            if(err_angle < 0 or err_angle > 3.1457):
                mot_msg.angular.z =  0.6
                mot_msg.linear.x = 0.0
                mot_msg.linear.y = 0.0

            elif err_angle > 0 or err_angle > -3.1457:
                mot_msg.angular.z = - 0.6
                mot_msg.linear.x = 0.0
                mot_msg.linear.y = 0.0
        else:
            mot_msg.angular.z = 0.0
            if(np.abs(err_x) > 0.15): # envi 2.9 deg
                if(err_x > 0):
                    mot_msg.linear.x = 0.35
                    mot_msg.linear.y = 0.0

            else:
                mot_msg.linear.x = 0.0
                mot_msg.linear.y = 0.0
                mot_msg.angular.z = 0.0
                mot_pub.publish(mot_msg)
                rospy.set_param('/waypoint_node/ready_to_go', False)
                print("Successful mission !")
                rospy.signal_shutdown('Quit') # Not the best way...
    else:
        mot_msg.linear.x = 0.0
        mot_msg.linear.y = 0.0
        mot_msg.angular.z = 0.0
    mot_pub.publish(mot_msg)

def run():
    rospy.loginfo("Waypoint starting up")
    rospy.init_node('Waypoint')
    rospy.Subscriber("/base_link_odom_camera_is1500", Odometry, turning_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

#*******************************************************************************
#   Main
#*******************************************************************************
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
    mot_msg.linear.x = 0.0
    mot_msg.linear.y = 0.0
    mot_msg.angular.z = 0.0
    ready_to_go = False
    success = True
    mot_pub.publish(mot_msg)
    print("end")
