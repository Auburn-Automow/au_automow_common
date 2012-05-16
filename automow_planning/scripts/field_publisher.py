#!/usr/bin/env python

"""
This ROS node takes the field survey file and publishes a
field polygon as a geometry_msgs/PolygonStamped for use in 
other nodes and for visualization in rviz.
"""

import roslib; roslib.load_manifest('automow_planning')
import rospy

from geometry_msgs.msg import PolygonStamped

class FieldPublisherNode(object):
    """
    This is a ROS node that is responsible for publishing the field.
    """
    def __init__(self):
        # Setup ROS node
        rospy.init_node('field_publisher')

        # Get ROS parameters
        self.field_file_name = rospy.get_param("~field_file_name", "field.csv")
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
        file_handle = None
        try:
            file_handle = open(self.field_file_name, 'r')
        except OSError as e:
            rospy.logerr("Failed to open field file: %s"%str(e))
            return False
        lines = file_handle.read()
        lines = lines.split("\n")
        self.field_polygon_msg = PolygonStamped()
        self.field_polygon_msg.header.stamp = rospy.Time.now()
        self.field_polygon_msg.header.frame_id = self.field_frame_id
        line_count = 0
        for line in lines:
            line_count += 1
            line = line.strip()
            if line == "":
                continue
            tokens = line.split(",")
            if len(tokens) != 3:
                rospy.logerr("Invalid field file at line %i"%line_count)
                return False
            (east, north, fix_type_id) = tokens
            self.field_polygon_msg.polygon.append(Point32(east, north, 0.0))
        return True

if __name__ == '__main__':
    fpn = FieldPublisherNode()
