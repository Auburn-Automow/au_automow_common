#!/usr/bin/env python
import roslib
roslib.load_manifest('automow_ekf')

import rospy
import math
import numpy as np
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler as qfe
from tf.transformations import quaternion_multiply, quaternion_about_axis

def wrapTo360(angle):
    return np.mod(angle+360,360)

def radAndWrap(angle):
    return np.mod(180 * angle/math.pi + 360, 360)

class ImuListener(object):
    def __init__(self):
        self.listener = tf.TransformListener()
        self.subscriber = rospy.Subscriber('/imu/data', Imu, self.imu_cb)
        self.publisher = rospy.Publisher('/imu/pose', PoseStamped) 
        
    def imu_cb(self, msg):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        
        # N-W-U to E-N-U is simply a rotation of -90 about the Z axis.
        tform = quaternion_about_axis(math.pi/2, (0,0,1))
        enu = quaternion_multiply(q,tform)
        new_msg = PoseStamped()
        new_msg.header = msg.header
        new_msg.pose.position.x = 0
        new_msg.pose.position.y = 0
        new_msg.pose.position.z = 0
        new_msg.pose.orientation.x = enu[0]
        new_msg.pose.orientation.y = enu[1]
        new_msg.pose.orientation.z = enu[2]
        new_msg.pose.orientation.w = enu[3]
        self.publisher.publish(new_msg)


if __name__ == '__main__':
    rospy.init_node('imu_listener')
    listener = ImuListener()
    
    rospy.spin()

