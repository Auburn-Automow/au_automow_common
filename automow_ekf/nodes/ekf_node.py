#!/usr/bin/env python
import roslib; roslib.load_manifest('automow_ekf')
import rospy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, PoseStamped
from ax2550.msg import StampedEncoders

from tf.transformations import euler_from_quaternion
import tf

from automow_ekf import AutomowEKF

import numpy as np
import threading

class AutomowEKF_Node:
    def __init__(self):
        # Load Parameters
        self.odom_used = rospy.get_param("~odom_used",True)
        self.imu_used = rospy.get_param("~imu_used",True)
        self.gps_used = rospy.get_param("~gps_used",True)
        self.decimate_ahrs = rospy.get_param("~decimate_ahrs_by_factor",0)
        self.publish_rate = 1.0/rospy.get_param("~output_publish_rate", 25)
        self.time_delay = rospy.get_param("~time_delay",0.01)
        self.output_frame = rospy.get_param("~output_frame","odom_combined")
        
        self.location_initilized = False
        self.heading_initilized = False
        self.ahrs_count = 0
        
        # Class Variables
        self.ekf = AutomowEKF.fromDefault()
        self.v = 0
        self.w = 0
        
        # Subscribers
        if self.odom_used:
            rospy.loginfo("Adding Encoders to the EKF")
            self.enc_sub = rospy.Subscriber("encoders",StampedEncoders,self.encoders_cb)
        else: rospy.loginfo("You aren't using the wheel encoders, expect bad results")
        rospy.loginfo("Adding IMU to the EKF")
        self.imu_sub = rospy.Subscriber("imu/data",Imu,self.imu_cb)
        rospy.loginfo("Adding GPS to the EKF")
        self.gps_sub = rospy.Subscriber("gps/odometry",Odometry,self.gps_cb)
        # Publishers
        self.odom_pub = rospy.Publisher("ekf/odom",Odometry)
        
        self.odometry_timer = threading.Timer(self.publish_rate, self.odometry_cb)
        self.odometry_timer.start()
    
    def odometry_cb(self):
        if rospy.is_shutdown():
            return
        self.odometry_timer = threading.Timer(self.publish_rate, self.odometry_cb)
        self.odometry_timer.start()
        if not self.location_initilized or not self.heading_initilized:
            return
        
        msg = Odometry()
        # Header
        msg.header.stamp = rospy.Time.now() - rospy.Duration(self.time_delay)
        msg.header.frame_id = self.output_frame
        # Position
        msg.pose.pose.position = Vector3(self.ekf.getEasting(),self.ekf.getNorthing(),0)
        quat = tf.transformations.quaternion_from_euler(0,0,self.ekf.getYaw())
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        #TODO: Add covariance to the Odometry message
        # Velocity
        msg.twist.twist.linear = Vector3(self.v,0,0)
        msg.twist.twist.angular = Vector3(0,0,self.w)
        self.odom_pub.publish(msg)
    
    def encoders_cb(self,data):
        u = np.array([data.encoders.left_wheel, data.encoders.right_wheel], \
                dtype=np.double)
        (self.v,self.w) = self.ekf.timeUpdate(u, data.header.stamp.to_sec())
        
        br = tf.TransformBroadcaster()
        br.sendTransform((self.ekf.getEasting(), self.ekf.getNorthing(), 0),
                         tf.transformations.quaternion_from_euler(0, 0, self.ekf.getYaw()),
                         rospy.Time.now(),
                         "base_footprint",
                         self.output_frame)
        return

    def imu_cb(self,data):
        if self.decimate_ahrs not in [0,1]:
            if self.ahrs_count % self.decimate_ahrs == 0:
                self.ahrs_count = 1
            else:
                self.ahrs_count += 1
                return
        if not self.imu_used:
            if self.heading_initilized:
                return
        if not self.heading_initilized:
            self.heading_initilized = True
        (r,p,yaw) = euler_from_quaternion([data.orientation.x, data.orientation.y, \
                data.orientation.z, data.orientation.w])
        # This is supposed to correct for the coordinate differences
        self.ekf.measurementUpdateAHRS(yaw-np.pi/2.0)    
        return

    def gps_cb(self,data):
        if not self.gps_used:
            if self.location_initilized:
                return
        if not self.location_initilized:
            self.location_initilized = True
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
        self.ekf.measurementUpdateGPS(y, covar)
        return


def main():
    rospy.init_node('ekf_node')
    ekf_node = AutomowEKF_Node()
    
    rospy.spin()
    
    ekf_node.odometry_timer.cancel()

if __name__ == '__main__':
    main()
