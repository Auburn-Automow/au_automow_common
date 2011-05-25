#! /usr/bin/env python

from __future__ import with_statement

import roslib; roslib.load_manifest('automow_calibration')
import rospy

from ax2550.msg import StampedEncoders
from geometry_msgs.msg import Twist

import threading

class TestAccels:
    enc_linear_accel = 0
    enc_angular_accel = 0
    prev_v = 0
    prev_w = 0
    def __init__(self):
        self.pub_cmdvel = rospy.Publisher('cmd_vel', Twist)
        self.sub_odom = rospy.Subscriber('encoders', StampedEncoders, self.enc_cb)

        self.f = open('ang_accel_data.csv','w')
        self.R_l = rospy.get_param("radius_left",0.159)
        self.R_r = rospy.get_param("radius_right",0.159)
        self.d = rospy.get_param("wheelbase_width",0.5461)

        self.prev_time = rospy.Time()
        self.lock = threading.Lock()

    def enc_cb(self,msg):
        with self.lock:
            left = msg.encoders.left_wheel
            right = msg.encoders.right_wheel

            dt = (msg.header.stamp - self.prev_time).to_sec()
            self.prev_time = msg.header.stamp

            v = self.R_l/2.0 * left + self.R_r/2.0 * right
            w = self.R_r/self.d * right - self.R_l/self.d * left

            self.enc_linear_accel = v - self.prev_v/dt
            self.enc_angular_accel = w - self.prev_w/dt

            self.prev_v = v
            self.prev_w = w
            self.f.write(str(v)+ "," + str(w)+ "\n")

    def test_linear_accel(self, speed, duration):
        rospy.loginfo('Testing Linear Acceleration with top speed: %f m/s'%speed)

        msg = Twist()
        msg.linear.x = speed
        stop = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < stop:
            self.pub_cmdvel.publish(msg)
            with self.lock:
                rospy.loginfo(self.enc_linear_accel)
        self.pub_cmdvel.publish(Twist())
        rospy.loginfo('Linear Acceleration Test Complete')

    def test_angular_accel(self,speed, duration):
        rospy.loginfo('Testing Angular Acceleration with top speed: %f rad/s'%speed)
        msg = Twist()
        msg.angular.z = speed
        stop = rospy.Time.now()+rospy.Duration(duration)
        while rospy.Time.now() < stop:
            self.pub_cmdvel.publish(msg)
            with self.lock:
                rospy.loginfo(self.enc_angular_accel)
        self.pub_cmdvel.publish(Twist())
        rospy.loginfo('Angular Acceleration Test Complete')


def main():
    rospy.init_node('TestAccels')
    robot = TestAccels()

    for speed in (1.0,2.0):
        robot.test_linear_accel(speed,5.0)
        rospy.sleep(1)
    for speed in (0.1, 0.3, 0.7, 1.0, 1.5, 2.0, 5.0):
        robot.test_angular_accel(speed,5.0)
        rospy.sleep(1)


if __name__ == '__main__':
    main()

