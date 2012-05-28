#!/usr/bin/env python

import roslib
roslib.load_manifest('automow_fence')

import threading

import rospy
import dynamic_reconfigure.server

from automow_fence.cfg import ControllerConfig
from automow_fence.controller import Controller, LineModel

from nav_msgs.msg import Odometry


class AutomowFence(object):
    def __init__(self):
        dynamic_reconfigure.server.Server(ControllerConfig, self.reconfigure)
        self.controller = Controller(0.5, 0.5)
        self.path = None
        self.tracking = False

        self.controller.heading_error_gain = rospy.get_param("~k_heading", 1.0)
        self.controller.lateral_error_gain = rospy.get_param("~k_lateral", 1.0)
        self.controller.max_v = rospy.gen_param("max_v", 1.0)
        self.controller.max_w = rospy.gen_param("max_w", 1.0)

        self.odom = rospy.Subscriber('/ekf/odom', Odometry, self.odom_cb)

        self.control_timer = threading.Timer(self.publish_rate,
                                             self.controller_cb)
        self.control_timer.start()

    def set_path(self, path):
        self.path = path
        self.tracking = True

    def reconfigure(self, config, level):
        self.controller.heading_error_gain = config['k_heading']
        self.controller.lateral_error_gain = config['k_lateral']
        self.controller.max_v = config['max_v']
        self.controller.max_w = config['max_w']
        return config

    def odom_cb(self, msg):
        pass

    def controller_cb(self):
        if rospy.is_shutdown():
            return
        self.control_timer = threading.Timer(self.publish_rate,
                                             self.controller_cb)
        self.control_timer.start()

    def spin():
        while not rospy.is_shutdown():
            time.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('automow_fence')
    af = AutomowFence()
    af.spin()
