#!/usr/bin/env python
import roslib; roslib.load_manifest('automow_tests')
import rospy
import sys
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped, Point
import tf
import math

global euler_pub 
euler_pub = None

def imuDataReceived(data):
    global euler_pub 

    angles = tf.transformations.euler_from_quaternion(
            [data.orientation.x, 
            data.orientation.y, 
            data.orientation.z, 
            data.orientation.w])
    imu_msg = Imu()
    
    yaw = wrapToPi(angles[2] - math.pi/2)

    msg = Vector3()
    msg.x = angles[0]
    msg.y = angles[1]
    msg.z = angles[2]
    euler_pub.publish(msg)

if __name__=='__main__':
    rospy.init_node('imu_eulerizer', anonymous=True)
    rospy.Subscriber('/imu/data',Imu,imuDataReceived)

    euler_pub = rospy.Publisher('/imu/euler',Vector3)
    rospy.spin()
