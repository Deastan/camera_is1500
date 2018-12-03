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

#global mot_pub
# mot_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1) # 100 to 1

rate = 0

def publisher_sevenSense(sevenSense_pub, listener):
    
    global rate

    print("Start publish")

    print("Issue: 1")
    (trans,rot) = listener.lookupTransform('/cam_3_optical_frame', '/imu', rospy.Time(0))
    print("Issue: 2")
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "/map"#self.frame_id # i.e. '/odom'
    msg.child_frame_id = "/base_link"#self.child_frame_id # i.e. '/base_footprint'
    msg.pose.pose.position = Point(trans[0], trans[1], trans[2])
    msg.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    sevenSense_pub.publish(msg)
    print("End publish")
    rate.sleep()

def run():
    global sevenSense_pub
    global rate
    rospy.loginfo("Transfromation starting up")
    rospy.init_node('Transfromation')
    sevenSense_pub = rospy.Publisher('/odom_sevenSense', Odometry)

    listener = tf.TransformListener()
    rate = rospy.Rate(10)
    # rospy.Subscriber("/base_link_odom_camera_is1500", Odometry, turning_callback)
    publisher_sevenSense(sevenSense_pub, listener)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

 #def main_loop(self):
#    while not rospy.is_shutdown():
#        self.rate.sleep()

#*******************************************************************************
#   Main
#*******************************************************************************
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass

    print("end")
