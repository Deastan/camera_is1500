#! /usr/bin/env python

#Basic imports
import sys
import rospy
import time
import tf
import numpy as np


from geometry_msgs.msg import Quaternion, Point, Pose, Twist, Vector3
from std_msgs.msg import Empty, Float32
from math import radians, pow

import tf.transformations
# from std_msgs.msg import Empty
from std_msgs.msg import String


#Publish directly what we need for the motor
global motor_pub
motorpub = rospy.Publisher('/motor_state', String, queue_size=100)

global mot_pub
mot_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
global mot_msg

def turning_callback(msg):
    global state

    if()


    publish a twist command for the motor
    mot_msg.linear.x = 0.25
    mot_msg.linear.y = 0
    mot_msg.angular.z = 1
    mot_pub.publish(mot_msg)
    #rospy.loginfo("Received a /cmd_vel message!")
    #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    # Do velocity processing here:
    # Use the kinematics of your robot to map linear and angular velocities into motor commands
    # for the physics based model:
    #v_l = msg.linear.x - msg.angular.z*robot_width/2#left wheel request of velocity
    #v_r = msg.linear.x + msg.angular.z*robot_width/2#right wheel request of velocity
    # Then set your wheel speeds (using wheel_left and wheel_right as examples)
    #set_left(v_l)
    #set_right(v_r)
    # if(state=="running" or state=="running_sonar_off" or state=="critical_obstacle"):
    #     if(msg.angular.z>0):
    #         leftTurn()
    #         #rospy.loginfo("angular left")
    #     elif(msg.angular.z<0):
    #         rightTurn()
    #         #rospy.loginfo("angular right")
    #     else:
	#     noTurn()





# def main_timer_callback(event):
#     global state
#     global motor_state
#     global sonar_off
#     global red_state
#     global green_state
#     global greet_timer
#     global start_time
#     global mot_msg
#     mot_msg= Twist()
#     if(state=="ready" and motor_state=="off"):
#         motor_on()
#         if(motor_state=="on"): #retries motor on command
#             if(sonar_off):
#                 state="running_sonar_off"
#             else:
# 	        state="running"
#         rospy.loginfo("motors commanded on, put in run state")
#     elif(state=="ready" and motor_state=="on"): #catch case where code failed but motor is still on
#         if(sonar_off):
#             state="running_sonar_off"
# 	    red_state=False
#             green_state=True
#         else:
#             state="running"
#             red_state=False
#             green_state=True
#     statepub.publish(state)
#     motor_pub="motor state: " + motor_state
#     motorpub.publish(motor_pub)
#     #publish a twist command for the motor
#     #mot_msg.linear.x = 0.25
#     #mot_msg.linear.y = 0
#     #mot_msg.angular.z = 1
#     #mot_pub.publish(mot_msg)
#     #add elapsed system time
#     greet_timer+=time.time()-start_time
#     start_time=time.time()

def run():
    self.ready_to_go = rospy.get_param('~ready_to_go', False)# We won't the one from the GPS neither wheel_odom
    self.target_x = rospy.get_param('~target_x', 3.7)
    self.target_y = rospy.get_param('~target_x', -1.5)

    start_time = time.time()
    rospy.loginfo("Waypoint starting up")

    #basic program code

    rospy.init_node('Waypoint')

    rospy.Subscriber("/base_link_odom_camera_is1500", Odometry, turning_callback)

    # timer = rospy.Timer(rospy.Duration(0.1), main_timer_callback)

    # blinkertimer = rospy.Timer(rospy.Duration(0.5), blinker_state_callback) #set the duration of this callback to set the speed of the blink

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    timer.shutdown()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
pass
