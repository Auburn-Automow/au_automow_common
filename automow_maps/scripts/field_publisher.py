#!/usr/bin/env python

"""
This ROS node takes the field survey file and publishes a
field polygon as a geometry_msgs/PolygonStamped for use in 
other nodes and for visualization in rviz.
"""

import roslib; roslib.load_manifest('automow_maps')
import rospy

from geometry_msgs.msg import PolygonStamped, Point32, Polygon

class FieldPublisherNode(object):
    """
    This is a ROS node that is responsible for publishing the field.
    """
    def __init__(self):
        # Setup ROS node
        rospy.init_node('field_publisher')

        # Get ROS parameters
        self.field_polygon = rospy.get_param("~field_polygon")
        self.field_frame_id = rospy.get_param("~field_frame_id", "odom")

        # Setup publishers and subscribers
        field_pub = rospy.Publisher('/field_shape', PolygonStamped, latch=True)

        # Read the field in
        if self.read_field_file():
            # Publish the msg once, it is latched so no need to repeat
            field_pub.publish(self.field_polygon_msg)
            # Spin
            rospy.spin()

    def read_field_file(self):
        self.field_polygon_msg = PolygonStamped()
        self.field_polygon_msg.header.stamp = rospy.Time.now()
        self.field_polygon_msg.header.frame_id = self.field_frame_id
        polygon_points = []
        point_count = 0
        for point in self.field_polygon:
            point_count += 1
            if point['fix_type'] < 3:
                rospy.logwarn('Point %i has a low quality fix type of %i'
                              % (point_count, point['fix_type']))
            (easting, northing) = (point['easting'], point['northing'])
            polygon_points.append(Point32(float(easting), float(northing), 0))
        self.field_polygon_msg.polygon = Polygon(polygon_points)
        return True

if __name__ == '__main__':
    fpn = FieldPublisherNode()
