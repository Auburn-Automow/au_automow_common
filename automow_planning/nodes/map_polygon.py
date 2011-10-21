#!/usr/bin/env python

"""
The map_polygon node will handle various map related utilities.

* Publish a polygon shape for a given field csv.
"""

import roslib
roslib.load_manifest('automow_planning')

import os
import rospy
import threading

from math import ceil, floor

# ROS msgs
from std_msgs.msg import Header
from geometry_msgs.msg import PolygonStamped, Polygon

def load_field_shape(file_name, meters_per_cell, image_padding):
    """
    Opens a csv file and reads the shape of the field, returns a polygon.
    """
    csv_file = open(file_name, 'r')

    contents = csv_file.read()
    lines = contents.split("\n")

    points = []
    point32s = []
    for line in lines:
        line = line.strip()
        if line == "":
            continue
        sep_values = line.split(",")
        if len(sep_values) == 3:
            (east,north,fix) = sep_values
            points.append((float(east) / meters_per_cell,
                           float(north) / meters_per_cell))
            point32s.append(Point32(float(east), float(north), 0.0))
        else:
            rospy.logerr("Error unpacking field csv...")
            rospy.signal_shutdown("Error unpacking field csv...")
            return

    field_xs, field_ys = zip(*points)

    # (east, north)
    offset = (ceil(min(field_xs)) - image_padding,
              floor(max(field_ys)) + image_padding)
    size = (ceil(max(field_xs)) - floor(min(field_xs)) + (2 * image_padding),
            ceil(max(field_ys)) - floor(min(field_ys)) + (2 * image_padding))

    rospy.loginfo("Map Offset East: %f cells (%f meters)" %
                     (offset[0], offset[0] * meters_per_cell))
    rospy.loginfo("Map Offset North: %f cells (%f meters)" %
                     (offset[1], offset[1] * meters_per_cell))
    rospy.loginfo("Map Size East: %f cells (%f meters)" % 
                     (size[0], size[0] * meters_per_cell))
    rospy.loginfo("Map Size North: %f cells (%f meters)" %
                     (size[1], size[1] * meters_per_cell))

    poly_msg = PolygonStamped(Header(), Polygon(point32s))
    return poly_msg

def poly_publisher(poly_pub, shape_pub_rate, poly_msg):
    """
    Publishes the polygon.
    """
    if not rospy.is_shutdown():
        poly_pub_timer = threading.Timer(1.0 / shape_pub_rate,
                                         polyPublishHandler,
                                         args=(poly_pub,
                                               shape_pub_rate,
                                               poly_msg))
        poly_pub_timer.start()
    poly_msg.header.stamp = rospy.Time.now()
    poly_pub.publish(poly_msg)


def main():
    rospy.init_node("map_polygon")
    field_file      = rospy.get_param("~field_csv_file", "/tmp/field.csv")
    meters_per_cell = rospy.get_param("~meters_per_cell", 0.4)
    image_padding   = rospy.get_param("~image_padding", 4)
    frame_id        = rospy.get_param("~field_frame_id", "odom_combined")
    shape_pub_rate  = rospy.get_param("~field_shape_publish_rate", 1.0)
    if not os.path.exists(field_file):
        rospy.logerr("Specified field csv file does not exist: %s" %
                         field_file)
        rospy.logerr("Resolved name: %s" % rospy.resolve_name("field_csv_file"))
        return

    poly_msg = load_field_shape(field_file, meters_per_cell, image_padding)
    poly_msg.header.frame_id = frame_id

    poly_pub = rospy.Publisher("field_shape", PolygonStamped)
    poly_pub_timer = threading.Timer(1.0 / shape_pub_rate,
                                     polyPublishHandler,
                                     args=(poly_pub, shape_pub_rate, poly_msg))
    poly_pub_timer.start()



if __name__ == '__main__':
    main()


