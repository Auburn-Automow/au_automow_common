#!/usr/bin/env python
# encoding: utf-8

import roslib; roslib.load_manifest('ax2550odom')
import rospy
from rospy.rostime import Time

from ax2550.msg import Encoder 
from nav_msg.msg import Odometry

odom_pub = None

def encoderDataReceived(data):
    """Called when encoder data is received"""
    global odom_pub
    left = data.left # left encoder ticks
    right = data.right # right encoder ticks
    ### Do math here
    
    ### Insert math into Odom msg so it can be published
    odom_msg = Odometry()
    
    ### Publishing Odom_msg
    odom_pub.publish(odom_msg)

def ax2550EncodersListener():
    """Main loop"""
    global odom_pub
    rospy.init_node('ax2550_odom', anonymous=True)
    rospy.Subscriber('motor_control_encoders', Encoder, self.encoderDataReceived)
    
    odom_pub = rospy.Publisher('ax2550_odometry', Odometry)
    
    rospy.spin()
    
if __name__ == '__main__':
    ax2550EncodersListener()