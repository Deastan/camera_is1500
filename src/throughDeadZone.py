#! /usr/bin/env python

# ET1 Deadzone for camera_is1500
# Jonathan Burkhard, Kyburz 2018
# Documentations : Not exist yet

# TODO
# -

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

# global mot_pub
mot_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1) # 100 to 1
# global mot_msg # Contain the msg
mot_msg = Twist()
# Distance to do
distance = 2.0
init = False

def turning_callback(msg):
    # start = time.time()
    # start_interm = time.time()
    # Re-assignement inside func
    global distance
    # target = [3.5,0.5]
    global mot_msg
    rate = rospy.Rate(130)
    global init
    offset_x = 0.0
    if init == False:
        offset_x = msg.pose.pose.position.x
        init = True
    # robot_y = msg.pose.pose.position.y
    # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    # u = [np.cos(yaw), np.sin(yaw)] # direction of the robot
    # v = [target[0] -robot_x, target[1] - robot_y] # vector btw robot and target
    # angle = np.arctan2(v[1], v[0])
    # err_angle = angle - yaw # angle(u, v)
    # err_x = length(v) # distance btw robot un target
    #print('Yaw: ',yaw/2/3.14*360, ', angle target: ', angle/2/3.14*360, ', error angle: ', err_angle/2/3.14*360)
    #print("Err_angle : ", err_angle, "Err_x : ", err_x, '\n', "Robot position : (", robot_x, ", ", robot_y, ") ")
    # end = time.time()
    else:

        error_x = msg.pose.pose.position.x-offset_x
        print("Distance to do: ", distance, ", error_x : ", error_x)

        # First change the angle to have the point in front of the robot and
        # after move forward
        if(error_x < distance ): # envi 2*2.9g

            mot_msg.angular.z = 0.0
            mot_msg.linear.x = 0.5
            mot_msg.linear.y = 0.0
            mot_pub.publish(mot_msg)

        else:
            mot_msg.linear.x = 0.0
            mot_msg.linear.y = 0.0
            mot_msg.angular.z = 0.0
            mot_pub.publish(mot_msg)

            rospy.signal_shutdown('Quit') # Not the best way...

    rate.sleep()
        #while not rospy.is_shutdown():
            #rate.sleep()

        # end = time.time()
        #print('End function duration:', 1/(end - start))

def run():
    rospy.loginfo("Waypoint starting up")
    rospy.init_node('DeadZone')
    #self.rate = rospy.Rate(10)
    rospy.Subscriber("/odom", Odometry, turning_callback)

    #rate.sleep()
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
    # To be sure it is stop
    mot_msg.linear.x = 0.0
    mot_msg.linear.y = 0.0
    mot_msg.angular.z = 0.0

    mot_pub.publish(mot_msg)
    print("end")
