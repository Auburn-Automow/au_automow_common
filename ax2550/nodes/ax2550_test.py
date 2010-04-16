#!/usr/bin/env python

import roslib; roslib.load_manifest('ax2550')
import rospy
from std_msgs.msg import String
from ax2550.srv import Move

def testMove():
    """Tests the Move srv provided by ax2550_driver.py"""
    rospy.wait_for_service('move')
    try:
        move = rospy.ServiceProxy('move', Move)
        resp1 = move(0, 1)
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def talker():
    pub = rospy.Publisher('motor_control', String)
    rospy.init_node('ax2550_test')
    while not rospy.is_shutdown():
        testMove()
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
