#!/usr/bin/env python
import roslib; roslib.load_manifest('automow_ekf')
import rospy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from ax2550.msg import StampedEncoders

from tf.transformations import euler_from_quaternion
import tf

from automow_ekf import AutomowEKF

import numpy as np

ekf = None
odom_pub = None
v = None
w = None

def encodersCallback(data):
    global ekf
    u = np.array([data.encoders.left_wheel, data.encoders.right_wheel], \
            dtype=np.double)

    (v,w) = ekf.timeUpdate(u, data.header.stamp.to_sec())

def imuCallback(data):
    global ekf, odom_pub
    (r,p,yaw) = euler_from_quaternion([data.orientation.x, data.orientation.y, \
            data.orientation.z, data.orientation.w])
    ekf.measurementUpdateAHRS(yaw-np.pi/2.0)    
    
    br = tf.TransformBroadcaster()
    br.sendTransform((ekf.getEasting(), ekf.getNorthing(), 0),
                     tf.transformations.quaternion_from_euler(0, 0, ekf.getYaw()),
                     rospy.Time.now(),
                     "base_footprint",
                     "odom_combined")
    
    msg = Odometry()
    # Header
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom_combined"
    # Position
    msg.pose.pose.position = Vector3(ekf.getEasting(),ekf.getNorthing(),0)
    quat = tf.transformations.quaternion_from_euler(0,0,ekf.getYaw())
    msg.pose.pose.orientation.x = quat[0]
    msg.pose.pose.orientation.y = quat[1]
    msg.pose.pose.orientation.z = quat[2]
    msg.pose.pose.orientation.w = quat[3]
    # Pose Covariance
    cov = np.diag(ekf.P)
    msg.pose.pose.covariance = np.diag(np.array((cov[0], cov[1], 0, 0, 0, cov[2])))
    # Velocity
    msg.twist.twist.linear = Vector3(v,0,0)
    msg.twist.twist.angular = Vector3(0,0,w)
    odom_pub.publish(msg)

def gpsCallback(data):
    global ekf
    y = np.array([data.pose.pose.position.x, data.pose.pose.position.y], \
            dtype=np.double)
    # The Kalman Filter expects a time-varying measurement noise matrix.  The GPS 
    # also has a nasty tendency to over-estimate the accuracy of the position, so
    # we create a floor of 2cm variance to prevent it from getting "too accurate"
    e_covar = data.pose.covariance[0]
    if e_covar < 0.004:
        e_covar = 0.004
    n_covar = data.pose.covariance[4]
    if n_covar < 0.004:
        n_covar = 0.004
    covar = np.diag(np.array([e_covar, n_covar]))
    ekf.measurementUpdateGPS(y, covar)

def ekf_node():
    global ekf, odom_pub
    rospy.init_node('ekf_node', anonymous=True)
    
    ekf = AutomowEKF.fromDefault()
    
    rospy.Subscriber("encoders", StampedEncoders, encodersCallback)
    rospy.Subscriber("imu/data", Imu, imuCallback)
    rospy.Subscriber("gps/odometry", Odometry, gpsCallback)
    
    odom_pub = rospy.Publisher("ekf/odom", Odometry)
    
    rospy.spin()

if __name__ == '__main__':
    ekf_node()
