#!/usr/bin/env python
# encoding: utf-8

import roslib; roslib.load_manifest('ax2550')
import rospy
from rospy.rostime import Time

from ax2550.msg import Encoder 
from nav_msg.msg import Odometry
from geometry_msg.msg

import tf
import math

previous_time = rospy.Time.now()
current_time = rospy.Time.now()

odom_pub = None
odom_broadcaster = None

local_offset = (0,0,0)
global_offset = (0,0,0)

wheel_base_width = 0.6477 #meters = 25.5 in
wheel_base_length =  0.9144 #meters =  36 in
wheel_diameter = 0.3175 #meters = 13 in
wheel_circum = math.pi * wheel_diameter
encoder_resolution = 250*4 #motor controller reads * 4 (quadrature)
prev_theta=0

def encoderDataReceived(data):
    """Called when encoder data is received"""
    global odom_pub
    global odom_broadcaster
    
    left = data.left * wheel_circum/encoder_resolution # left encoder ticks
    right = data.right * wheel_circum/encoder_resolution # right encoder ticks
    ### Do math here
    
    """if(left == right):
        x = left * math.sin(theta)
        y = left * math.cos(theta)
    else:
        mult = wheel_base_width/2*(right+left)/(right-left) 
        x = mult*math.sin((right-left)/wheel_base_width + theta)-math.sin(theta)
        y = -mult*math.cos((right-left)/wheel_base_width + theta)-math.cos(theta)"""
    
    sbar = (right+left)/2
    theta = (right-left)/(2*wheel_base_width) + prev_theta
    x = sbar * math.cos(theta) + prev_x
    y = sbar * math.sin(theta) + prev_y
    
    (prev_x,prev_y,prev_theta) = (x,y,theta)
    
    ### Insert math into Odom msg so it can be published
    odom_msg = Odometry()
    odom_msg.header.frame_id="/world"
    odom_msg.pose.position.x = x
    odom_msg.pose.position.y = y
    odom_msg.pose.position.z = 0
    odom_msg.pose.orientation.x = 0
    odom_msg.pose.orientation.y = 0
    odom_msg.pose.orientation.z = 0
    odom_msg.pose.orientation.w = 1
    
    
    ### Publishing Odom_msg
    odom_pub.publish(odom_msg)


def ax2550EncodersListener():
    """Main loop"""
    global odom_pub
    global odom_broadcaster
    
    rospy.init_node('ax2550_odom', anonymous=True)
    rospy.Subscriber('motor_control_encoders', Encoder, self.encoderDataReceived)
    
    odom_pub = rospy.Publisher('ax2550_odometry', Odometry)
    rospy.spin()
    
if __name__ == '__main__':
    ax2550EncodersListener()