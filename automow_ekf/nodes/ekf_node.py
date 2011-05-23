#!/usr/bin/env python
import roslib; roslib.load_manifest('automow_ekf')
import rospy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ax2550.msg import StampedEncoders

from tf.transformations import euler_from_quaternion
import tf

from automow_ekf import AutomowEKF

import matplotlib

import numpy as np

ekf = None

def encodersCallback(data):
    global ekf
    u = np.array([data.encoders.left_wheel, data.encoders.right_wheel], dtype=np.double)
    ekf.timeUpdate(u, data.header.stamp.to_sec())

def imuCallback(data):
    global ekf
    (r,p,yaw) = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    ekf.measurementUpdateAHRS(yaw)#, data.orientation_covariance[8])
    print ekf.getEasting(), ekf.getNorthing(), ekf.getYaw()

def ekf_node():
    global ekf
    rospy.init_node('ekf_node', anonymous=True)
    
    ekf = AutomowEKF.fromDefault()
    
    rospy.Subscriber("encoders", StampedEncoders, encodersCallback)
    rospy.Subscriber("imu/data", Imu, imuCallback)
    
    rospy.spin()

if __name__ == '__main__':
    ekf_node()