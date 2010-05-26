#!/usr/bin/env python
# encoding: utf-8

"""
ax2550_teleop.py - Used joy stick messages over /joy to send drive commands to the motor controller

Created by William Woodall on 2010-04-13.
"""
__author__ = "William Woodall"
__copyright__ = "Copyright (c) William Woodall"

###  Imports  ###

# ROS imports
import roslib; roslib.load_manifest('ax2550')
import rospy

# ROS msg and srv imports
from ax2550.srv import Move
from joy.msg import Joy

def move(speed, direction):
    """Calls the move srv on ax2550_driver.py"""
    try:
	speed /= 2
	direction /= 6
        move = rospy.ServiceProxy('move', Move)
        resp1 = move(speed, -1*direction)
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def joystickCallback(data):
    """Called everytime the joystick updates"""
    move(data.axes[1], data.axes[0])

def joystickListener():
    """Listens for Joystick signals"""
    rospy.init_node('ax2550_teleop', anonymous=True)
    s = rospy.Subscriber("joy", Joy, joystickCallback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    joystickListener()
