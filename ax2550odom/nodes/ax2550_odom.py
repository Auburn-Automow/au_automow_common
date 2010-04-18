#!/usr/bin/env python
# encoding: utf-8

import roslib; roslib.load_manifest('ax2550odom')
import rospy
from rospy.rostime import Time

from ax2550.msg import Encoder 
from nav_msg.msg import Odometry

def ax2550listener():
	rospy.init_node('ax2550odom', anonymous=True)
	rospy.Subscriber('motor_control_encoders', Encoder, self.encoderDataReceived)
	
	self.odom_pub = rospy.Publisher('ax2550_odometry', Odometry)
	