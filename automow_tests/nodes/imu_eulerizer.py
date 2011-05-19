#!/usr/bin/env python
import roslib; roslib.load_manifest('automow_tests')
import rospy
import sys
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped, Point
import tf
import math

global imu_pub
imu_pub = None

def imuDataReceived(data):
    global imu_pub

    angles = tf.transformations.euler_from_quaternion(
            [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    msg = Vector3()
    msg.x = angles[0]
    msg.y = angles[1]
    msg.z = angles[2]
    imu_pub.publish(msg)

if __name__=='__main__':
    rospy.init_node('imu_eulerizer', anonymous=True)
    rospy.Subscriber('/imu/data',Imu,imuDataReceived)

    imu_pub = rospy.Publisher('/imu/euler',Vector3)

    rospy.spin()
