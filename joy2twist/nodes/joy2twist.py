#!/usr/bin/env python
# encoding: utf-8

"""
joy2twist.py - Provides Twist messages given a joy topic.

Created by William Woodall on 2010-07-12.
"""
__author__ = "William Woodall"

###  Imports  ###

# ROS imports
import roslib; roslib.load_manifest('joy2twist')
import rospy

# ROS msg and srv imports
from joy.msg import Joy
from geometry_msgs.msg import Twist

# Python Libraries
import sys
import traceback

###  Variables  ###
LINEAR_SPEED = 0.5
ANGULAR_SPEED = 2.75

###  Classes  ###

class Joy2Twist(object):
    """Joy2Twist ROS Node"""
    def __init__(self):
        # Initialize the Node
        rospy.init_node("Joy2Twist")
        
        # Setup the Joy topic subscription
        self.joy_subscriber = rospy.Subscriber("joy", Joy, self.handleJoyMessage, queue_size=1)
        
        # Setup the Twist topic publisher
        self.twist_publisher = rospy.Publisher("cmd_vel", Twist)
        
        # Spin
        rospy.spin()
    
    def handleJoyMessage(self, data):
        """Handles incoming Joy messages"""
        msg = Twist()
        msg.linear.x = data.axes[1] * LINEAR_SPEED
        msg.angular.z = data.axes[0] * ANGULAR_SPEED
        self.twist_publisher.publish(msg)
    

###  If Main  ###
if __name__ == '__main__':
    try:
        Joy2Twist()
    except:
        rospy.logerr("Unhandled Exception in the joy2Twist Node:+\n"+traceback.format_exc())
