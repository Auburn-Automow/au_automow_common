#!/usr/bin/env python
import roslib; roslib.load_manifest('automow_ekf')
import rospy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from ax2550.msg import StampedEncoders
from magellan_dg14.msg import UTMFix
from automow_node.msg import Automow_PCB
from automow_ekf.msg import States

from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler as qfe
import tf

from automow_ekf import AutomowEKF

import numpy as np
import threading

class AutomowEKF_Node:
    def __init__(self):
        # Load Parameters
        self.odom_used = rospy.get_param("~odom_used", True)
        self.imu_used = rospy.get_param("~imu_used", True)
        self.gps_used = rospy.get_param("~gps_used", True)
        self.cutters_used = rospy.get_param("~cutters_used", False)

        self.decimate_ahrs = rospy.get_param("~decimate_ahrs_by_factor",0)
        self.publish_rate = 1.0/rospy.get_param("~output_publish_rate", 25)
        self.time_delay = rospy.get_param("~time_delay",0.0)
        
        self.output_tf = rospy.get_param("~output_tf", True)
        self.output_frame = rospy.get_param("~output_frame","odom_combined")
        self.output_states = rospy.get_param("~output_states",False)
        self.output_states_dir = rospy.get_param("~output_states_dir","/home/mjcarroll/.ros/")

        self.adaptive_encoders = rospy.get_param("~adaptive_encoders",False)
        self.publish_states = rospy.get_param("~publish_states",False)

        self.encoder_resolution = 1000 

        if self.output_states:
            import time
            self.output_states_file = self.output_states_dir + "ekf_states-" \
                    + time.strftime('%F-%H-%M') + ".csv"
            self.output_file = open(self.output_states_file,'w')

        self.location_initilized = False
        self.heading_initilized = False
        self.ahrs_count = 0
        
        # Class Variables
        self.ekf = AutomowEKF.fromDefault()
        self.v = 0
        self.w = 0
        self.filter_time = rospy.Time.now()
        self.cutter_l = 0
        self.cutter_r = 0
        
        # Subscribers
        if self.odom_used:
            rospy.loginfo("Adding Encoders to the EKF")
            self.enc_sub = rospy.Subscriber("/encoders",
                    StampedEncoders,
                    self.encoders_cb)
            self.enc_prev_time = rospy.Time.now()
    
        if self.imu_used:
            rospy.loginfo("Adding IMU to the EKF")
            self.imu_sub = rospy.Subscriber("/imu/data",
                    Imu,
                    self.imu_cb)
        else:
            rospy.loginfo("IMU will not be used.")
            self.ekf.measurementUpdateAHRS(0)
            self.filter_time = rospy.Time.now()
            self.heading_initilized = True

        if self.gps_used:
            rospy.loginfo("Adding GPS to the EKF")
            self.gps_sub = rospy.Subscriber("/magellan_dg14/odometry",
                    Odometry,
                    self.gps_cb)
            self.gps_fix_sub = rospy.Subscriber("/magellan_dg14/utm_fix",
                    UTMFix,
                    self.gps_fix_cb)
            self.gps_bad = False
        else:
            rospy.loginfo("GPS will not be used")
            y = np.array([0, 0], dtype=np.double)
            covar = np.diag(np.array([0.001, 0.001]))
            self.ekf.measurementUpdateGPS(y, covar)
            self.filter_time = rospy.Time.now()
            self.location_initilized = True

        if self.cutters_used: 
            rospy.loginfo("Subscribing to the cutter status")
            self.cut_sub = rospy.Subscriber("/automow_pcb/status",
                    Automow_PCB,
                    self.cutter_cb)
            rospy.loginfo("P matrix will change on cutter status")

        # Publishers
        self.odom_pub = rospy.Publisher("ekf/odom",Odometry)
        self.odometry_timer = threading.Timer(self.publish_rate,
                                              self.odometry_cb)
        self.odometry_timer.start()
        if self.publish_states:
            self.states_pub = rospy.Publisher("ekf/states",States) 
 
    def odometry_cb(self):
        if rospy.is_shutdown():
            return
        self.odometry_timer = threading.Timer(self.publish_rate,
                                              self.odometry_cb)
        self.odometry_timer.start()
        if not self.location_initilized or not self.heading_initilized:
            return
        
        msg = Odometry()
        # Header
        msg.header.stamp = self.filter_time  
        msg.header.frame_id = self.output_frame
        # Position
        msg.pose.pose.position = Vector3(self.ekf.getEasting(),
                                         self.ekf.getNorthing(), 0)
        quat = qfe(0, 0, self.ekf.getYaw())
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        #TODO: Add covariance to the Odometry message
        # Velocity
        msg.twist.twist.linear = Vector3(self.v,0,0)
        msg.twist.twist.angular = Vector3(0,0,self.w)
        self.odom_pub.publish(msg)
 

        if self.output_tf:
            br = tf.TransformBroadcaster()
            br.sendTransform((self.ekf.getEasting(), self.ekf.getNorthing(), 0),
                             qfe(0, 0, self.ekf.getYaw()),
                             self.filter_time,
                             "/base_footprint",
                             self.output_frame)
        if self.output_states:
            string = str(self.filter_time) + "," + \
                    self.ekf.getStateString()
            if self.cutters_used:
                string += str(self.cutter_l) + "," + \
                        str(self.cutter_r)
            string += "\n"
            self.output_file.write(string)

        if self.publish_states:
            self.states_pub.publish(States(self.ekf.getStateList(),
                                           self.ekf.getPList()))
        return
    
    def cutter_cb(self, data):
        # Cutters come on
        if (data.cutter_1 and not self.cutter_l) or \
                (data.cutter_2 and not self.cutter_r):
                    rospy.loginfo("Cutters came on! Fire ze missles!")
                    self.ekf.P[2,2] = 1e2
                    self.ekf.P[6,6] = 1e6
        if (not data.cutter_1 and self.cutter_l) or \
                (not data.cutter_2 and self.cutter_r):
                    rospy.loginfo("Cutters turned off! Fire ze missles!")
                    self.ekf.P[2,2] = 1e2
                    self.ekf.P[6,6] = 1e6
        self.cutter_l = int(data.cutter_1)
        self.cutter_r = int(data.cutter_2)
        return
        
    def encoders_cb(self,data):
        u = np.array([data.encoders.left_wheel, data.encoders.right_wheel], \
                dtype=np.double)

        # Multiply measurement by radians/encoder tick.
        u *= (2 * np.pi)/self.encoder_resolution
        u /= data.encoders.time_delta

        if self.adaptive_encoders:
            self.ekf.Q[0,0] = abs(u[0]) * 0.05
            self.ekf.Q[1,1] = abs(u[1]) * 0.05
        (self.v,self.w) = self.ekf.timeUpdate(u, data.header.stamp.to_sec())
        self.filter_time = data.header.stamp
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
        (r,p,yaw) = efq([data.orientation.x, data.orientation.y, \
                         data.orientation.z, data.orientation.w])
        # This is supposed to correct for the coordinate differences
        self.ekf.measurementUpdateAHRS(yaw + np.pi/2)    
        self.filter_time = data.header.stamp
        return

    def gps_fix_cb(self, data):
        if data.fix_type < 3:
            self.gps_bad = True
        else:
            self.gps_bad = False

    def gps_cb(self,data):
        if not self.location_initilized:
            self.location_initilized = True
        y = np.array([data.pose.pose.position.x, data.pose.pose.position.y], \
                dtype=np.double)
        # The Kalman Filter expects a time-varying measurement noise matrix.
        # The GPS also has a nasty tendency to over-estimate the accuracy of 
        # the position, so we create a floor of 2cm variance to prevent it 
        # from getting "too accurate"
        e_covar = data.pose.covariance[0]
        if e_covar < 0.004:
            e_covar = 0.004
        n_covar = data.pose.covariance[4]
        if n_covar < 0.004:
            n_covar = 0.004
        if not self.gps_bad:
            covar = np.diag(np.array([e_covar, n_covar]))
            self.ekf.measurementUpdateGPS(y, covar)
            self.filter_time = data.header.stamp
        return

def main():
    rospy.init_node('ekf_node')
    ekf_node = AutomowEKF_Node()
    rospy.spin()
    ekf_node.odometry_timer.cancel()

if __name__ == '__main__':
    main()
