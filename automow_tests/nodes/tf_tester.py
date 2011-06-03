#!/usr/bin/env python
import roslib 
roslib.load_manifest('automow_tests')

import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('tf_tester')

    listener = tf.TransformListener()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            (trans, _ ) = listener.lookupTransform('base_link','left_cutter',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            continue

        print trans
        rate.sleep()
