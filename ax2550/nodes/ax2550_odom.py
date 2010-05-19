#!/usr/bin/env python
# encoding: utf-8

import roslib; roslib.load_manifest('ax2550')
import rospy
from rospy.rostime import Time

from ax2550.msg import Encoder 
from nav_msgs.msg import Odometry

import tf
import math

odom_pub = None
wheel_base_width = 0.6477 #meters = 25.5 in
wheel_base_length =  0.9144 #meters =  36 in
wheel_diameter = 0.3175 #meters = 13 in
wheel_circum = math.pi * wheel_diameter
encoder_resolution = 250*4 #motor controller reads * 4 (quadrature)
prev_theta=0
prev_x=0
prev_y=0

MAX_DBL = 1e+100  

def encoderDataReceived(data):
    """Called when encoder data is received"""
    global odom_pub
    global wheel_base_width, wheel_base_length, wheel_diameter, wheel_circum,encoder_resolution
    global prev_theta,prev_x,prev_y
    left = data.left * wheel_circum/encoder_resolution # left encoder ticks
    right = data.right * wheel_circum/encoder_resolution # right encoder ticks
    
    ### Do math here
    if(right == left):
        v = right
        w = 0
    elif(left == -right):
        v = 0
        w = 2/wheel_base_width * right
    else:
        w = right-left/wheel_base_width
        v = 1/2 * (right+left)
    
    theta = w + prev_theta
    x = v * math.cos(theta) + prev_x
    y = v * math.sin(theta) + prev_y

    (prev_x,prev_y,prev_theta) = (x,y,theta)
 
    quat = tf.transformations.quaternion_from_euler(0,0,theta)
   
    ### Insert math into Odom msg so it can be published
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id="world"
    odom_msg.child_frame_id="base_odom"
    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.position.z = 0.0
    odom_msg.pose.pose.orientation.x = quat[0]
    odom_msg.pose.pose.orientation.y = quat[1]
    odom_msg.pose.pose.orientation.z = quat[2]
    odom_msg.pose.pose.orientation.w = quat[3]

    odom_msg.pose.covariance = [0.004, 0, 0, 0, 0, 0,
			        0, 0.004, 0, 0, 0, 0,
				0, 0, MAX_DBL, 0, 0, 0,
				0, 0, 0, MAX_DBL, 0, 0,
				0, 0, 0, 0, MAX_DBL, 0,
				0, 0, 0, 0, 0, 0.000289]
    odom_msg.twist.covariance = odom_msg.pose.covariance
 
    ### Publishing Odom_msg
    odom_pub.publish(odom_msg)

def ax2550EncodersListener():
    """Main loop"""
    global odom_pub
    
    rospy.init_node('ax2550_odom', anonymous=True)
    rospy.Subscriber('motor_control_encoders', Encoder, encoderDataReceived)
        
    odom_pub = rospy.Publisher('ax2550_odometry', Odometry)
    rospy.spin()
    
if __name__ == '__main__':
    ax2550EncodersListener()
