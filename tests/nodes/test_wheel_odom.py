#!/usr/bin/env python
import roslib; roslib.load_manifest('tests')
import rospy
from geometry_msgs.msg import Twist
def talker():
    pub = rospy.Publisher('/cmd_vel', Twist)
    rospy.init_node('test_wheel_odom')
    rospy.sleep(1.0)
    msg = Twist()
    msg.linear.x = 1.0
    msg.angular.z = 0
    pub.publish(msg)
    rospy.sleep(1.0)
    msg.linear.x = 0
    pub.publish(msg)

if __name__ == '__main__':
    talker()
    # try:
    #     talker()
    # except rospy.ROSInterruptException: pass