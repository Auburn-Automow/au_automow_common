#!/usr/bin/env python
import roslib; roslib.load_manifest('ax2550')
import rospy
from std_msgs.msg import String

import sys

print sys.path

def talker():
    pub = rospy.Publisher('motor_control', String)
    rospy.init_node('ax2550_test')
    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
