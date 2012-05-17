#!/usr/bin/env python
import roslib
roslib.load_manifest('automow_ekf')

import rospy
import math
import numpy as np
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

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
        self.publisher = rospy.Publisher('/imu/euler', Vector3) 
        
    def imu_cb(self, msg):
        try:
            (trans, rot) = self.listener.lookupTransform(msg.header.frame_id, '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr('Exception in TF lookup')
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        (r, p, y) = efq(q)
        
        # Raw Reading from the IMU:
        # 0 is north, and rotations in the CCW direction are positive.
        # I guess that means N-W-U
        rospy.loginfo("Raw: " + str(wrapTo360(180.0 * y / math.pi)))

        # N-W-U to E-N-U is simply a rotation of -90 about the Z axis.
        tform = quaternion_about_axis(math.pi/2, (0,0,1))
        enu = quaternion_multiply(q,tform)
        (r,p,y) = efq(enu)
        rospy.loginfo("ENU: " + str(radAndWrap(y)))

        y = (-y) + math.pi/2.0
        rospy.loginfo("NED: " + str(radAndWrap(y)))

        


if __name__ == '__main__':
    rospy.init_node('imu_listener')
    listener = ImuListener()
    
    rospy.spin()

