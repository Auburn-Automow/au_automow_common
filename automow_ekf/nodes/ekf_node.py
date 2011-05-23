#!/usr/bin/env python
import roslib; roslib.load_manifest('automow_ekf')
import rospy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ax2550.msg import StampedEncoders

from tf.transforms import euler_from_quaternion

from automow_ekf import AutomowEKF

ekf = None

def encodersCallback(data):
    global ekf
    ekf.timeUpdate(data.left_wheel, data.right_wheel, data.header.stamp.to_secs())

def imuCallback(data):
    global ekf
    (yaw,r,p) = euler_from_quaternion(data.orientation)
    ekf.measurementUpdateAHRS(yaw, data.orientation_covariance[8])

def ekf_node():
    global ekf
    rospy.init_node('ekf_node', anonymous=True)
    
    ekf = AutomowEKF()
    
    rospy.Subscriber("encoders", StampedEncoders, encodersCallback)
    rospy.Subscriber("imu/data", Imu, imuCallback)
    
    rospy.spin()

if __name__ == '__main__':
    ekf_node()