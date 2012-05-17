#!/usr/bin/env python

import roslib; roslib.load_manifest('error_ellipses')

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

pub = None

def callback(msg):
    global pub
    marker = Marker()
    marker.header = msg.header
    marker.ns = "error_ellipses"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose = msg.pose.pose
    from math import sqrt
    marker.scale.x = sqrt(msg.pose.covariance[0])*10
    marker.scale.y = sqrt(msg.pose.covariance[7])*10
    marker.scale.z = sqrt(msg.pose.covariance[14])*10
    marker.color.r = 0.0
    marker.color.b = 0.0
    marker.color.g = 1.0
    marker.color.a = 0.75
    marker.lifetime = rospy.Duration(1.0)
    pub.publish(marker)
    print(marker)

def main():
    global pub
    rospy.init_node('odom2tf')
    
    rospy.Subscriber("/magellan_dg14/odometry", Odometry, callback)
    pub = rospy.Publisher('visualization_marker', Marker)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass