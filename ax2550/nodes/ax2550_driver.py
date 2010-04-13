#!/usr/bin/env python
import roslib; roslib.load_manifest('ax2550')
import rospy
from std_msgs.msg import String

def commandReceived(data):
    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)

def motorControl():
    rospy.init_node('ax2550_driver', anonymous=True)
    rospy.Subscriber('motor_control', String, commandReceived)
    rospy.spin()

if __name__ == '__main__':
    motorControl()
