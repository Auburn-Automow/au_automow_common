#!/usr/bin/env python

"""
This ROS node takes the field polygon and the cutter 
positions and controls the cutting blades accordingly.

This node subscribes to the field_shape, which is published
as a geometry_msgs/PolygonStamped.  It also uses the tf provided
by the state published to figure out the position of the cutting
blades at any given time.

This node uses the shapely library to figure out if the cutters
are mostly in the field or out of the field.
"""

import roslib; roslib.load_manifest('automow_planning')
import rospy

from geometry_msgs.msg import PolygonStamped

import shapely.geometry as geo

class CutterControlNode(object):
    """
    This is a ROS node that is 
    responsible for controlling the cutters
    """
    def __init__(self):
        # Setup ROS node
        rospy.init_node('cutter_control')

        # Setup publishers and subscribers
        rospy.Subscriber('/field_shape', PolygonStamped, self.field_callback)

        # Setup initial variables
        self.field_shape = None

        # Spin
        rospy.spin()

    def field_callback(self, msg):
        # Convert the PolygonStamped into a shapely polygon
        temp_points = []
        for point in msg.polygon.points:
            temp_points.append(float(point))
        self.field_shape = geo.Polygon(temp_points)


if __name__ == '__main__':
    main()
