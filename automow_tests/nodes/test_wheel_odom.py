#!/usr/bin/env python
import roslib; roslib.load_manifest('tests')
import rospy
import sys
from geometry_msgs.msg import Twist
def talker():
    pub = rospy.Publisher('/cmd_vel', Twist)
    rospy.init_node('test_wheel_odom')
    rospy.sleep(1.0)
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0
    pub.publish(msg)
    
    rospy.sleep(1.0)
    
    speeds = [0.1*x for x in range(1,11)]
    for speed in speeds:
        print "Running speed: %f" % speed
        msg.linear.x = speed
        for i in range(1,31):
            print "."
            pub.publish(msg)
            rospy.sleep(12)
            if rospy.is_shutdown():
                sys.exit()
        print "done."
    
    msg.linear.x = 0
    pub.publish(msg)
    
    print "Done running."

if __name__ == '__main__':
    talker()
    # try:
    #     talker()
    # except rospy.ROSInterruptException: pass