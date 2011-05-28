#!/usr/bin/env python
import roslib; roslib.load_manifest('automow_tests')
import rospy
import sys
import threading
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped, Point
from nav_msgs.msg import Odometry
from magellan_dg14.msg import UTMFix
import tf
import math

global x,y,f,fixtype
f = None
x = 0
y = 0
fix_type = -1

def cb(data):
    global x,y,fixtype
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    rospy.loginfo("GPS Location: %f, %f"%(x,y))
    
def cb2(data):
    """docstring for data"""
    global fix_type
    fix_type = data.fix_type
    rospy.loginfo("GPS Fix Type: %f" % fix_type)

def spin():
    rospy.spin()

def go():
    global f,x,y,fixtype
    rospy.init_node('survey_field', anonymous=True)
    
    f = open('survey.csv', 'w')
    
    rospy.Subscriber('/gps/odometry',Odometry,cb)
    rospy.Subscriber('/dg14_driver/utm_fix',UTMFix,cb2)
    
    threading.Thread(target=spin).start()
    
    running = True
    
    while not rospy.is_shutdown() and running:
        stuff = raw_input()
        if stuff == 'e':
            running = False
            continue
        rospy.loginfo("Recording data to file: %f, %f, %f"%(x,y,fix_type))
        f.write(str(x)+","+str(y)+","+str(fix_type)+"\n")

if __name__=='__main__':
    global f,x,y,fixtype
    try:
        go()
    finally:
        f.close()
        rospy.signal_shutdown("do it")
