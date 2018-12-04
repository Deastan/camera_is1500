#! /usr/bin/env python

# Interface to get back the position of the robot from sevensSense A.G.
# Jonathan Burkhard, Kyburz 2018
# Documentations : Not exist yet

# TODO
# - Patch up the main
# - Create object ... and do it more nicer

# Basic imports
import sys
import rospy
import time
import tf
import numpy as np
import tf.transformations
from geometry_msgs.msg import Quaternion, Point, Pose, Twist, Vector3
from nav_msgs.msg import Odometry
from math import radians, pow

#*******************************************************************************
#   Main
#*******************************************************************************
if __name__ == '__main__':
    print("Start node: sevensense_tf_listener")
    rospy.init_node('sevensense_tf_listener')

    # declarate and init. Publisher
    sevenSense_pub = rospy.Publisher('/odom_sevenSense', Odometry)
    # declarate and init tf listener
    listener = tf.TransformListener()

    # Rate for the ros-loop
    rate = rospy.Rate(10.0)
    # Loop
    while not rospy.is_shutdown():
        try:
            # Get the lookupTransform to have the transfrom
            (trans,rot) = listener.lookupTransform('/base_link', '/map', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Create an odometry message
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/map"#self.frame_id # i.e. '/odom'
        msg.child_frame_id = "/base_link"#self.child_frame_id # i.e. '/base_footprint'
        msg.pose.pose.position = Point(trans[0], trans[1], trans[2])
        msg.pose.pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])

        # Publish the message
        sevenSense_pub.publish(msg)

        rate.sleep()

    print("End of the node")
# end of the main
