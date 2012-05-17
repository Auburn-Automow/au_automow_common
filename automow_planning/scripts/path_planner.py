#!/usr/bin/env python

"""
This ROS node is responsible for planning a path.

This node takes the field shape as a geometry_msgs/PolygonStamped
and publishes the path as a set of visualization markers.

Once the path has been generated the node can, by configuration or 
a service call, start feeding path waypoints as actionlib goals to move base.
"""

import roslib; roslib.load_manifest('automow_planning')
import rospy
import tf
import numpy as np

from geometry_msgs.msg import PolygonStamped, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import shapely.geometry as geo

class PathPlannerNode(object):
    """
    This is a ROS node that is responsible for planning and executing 
    the a path through the field.
    """
    def __init__(self):
        # Setup ROS node
        rospy.init_node('path_planner')

        # ROS params
        self.cut_spacing = rospy.get_param("~cut_spacing", 0.25)

        # Setup publishers and subscribers
        rospy.Subscriber('/field_shape', PolygonStamped, self.field_callback)
        self.path_marker_pub = rospy.Publisher('visualization_marker',
                                               MarkerArray,
                                               latch=True)
        '''TODO: setup actionlib'''

        # Setup initial variables
        self.field_shape = None
        self.field_frame_id = None
        self.path = None
        self.path_status = None
        self.path_markers = None
        self.start_path_following = False

        # Spin until shutdown or we are ready for path following
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and not self.start_path_following:
            rate.sleep()
        # If shutdown, return now
        if rospy.is_shutdown():
            return
        # Setup path following
        self.setup_path_following()
        # Iterate on path following and ROS spinning
        while not rospy.is_shutdown():
            self.step_path_following()
            rate.sleep()

    def field_callback(self, msg):
        """
        Handles new field polygons, has to be called 
        at least once before planning happens.
        """
        # Convert the PolygonStamped into a shapely polygon
        temp_points = []
        for point in msg.polygon.points:
            temp_points.append( (float(point.x), float(point.y)) )
        self.field_shape = geo.Polygon(temp_points)
        self.field_frame_id = msg.header.frame_id
        self.plan_path(self.field_shape)

    def plan_path(self, field_polygon):
        """
        This is called after a field polygon has been received.

        This uses the automow_planning.coverage module to plan a 
        coverage path using the field geometry.  The path consists of
        a series of waypoints.
        """
        # Get the rotation to align with the longest edge of the polygon
        from automow_planning.maptools import rotation_tf_from_longest_edge
        rotation = rotation_tf_from_longest_edge(field_polygon)
        # Rotate the field polygon
        from automow_planning.maptools import rotate_polygon_to
        transformed_field_polygon = rotate_polygon_to(field_polygon, rotation)
        # Decompose the rotated field into a series of waypoints
        from automow_planning.coverage import decompose
        transformed_path = decompose(transformed_field_polygon,
                                     width=self.cut_spacing)
        # Rotate the transformed path back into the source frame
        from automow_planning.maptools import rotate_from
        self.path = rotate_from(np.array(transformed_path), rotation)
        # Set the path_status to 'not_visited'
        self.path_status = []
        for waypoint in self.path:
            self.path_status.append('not_visited')
        # Visualize the data
        self.visualize_path(self.path, self.path_status)

    def visualize_path(self, path, path_status):
        """
        Publishes visualization Markers to represent the planned path.

        Publishes the path as a series of spheres connected by lines.
        The color of the spheres is set by the path_status parameter, 
        which is a list of strings of which the possible values are in
        ['not_visited', 'visiting', 'visited'].
        """
        # Get the time
        now = rospy.Time.now()
        # If self.path_markers is None, initialize it
        if self.path_markers == None:
            self.path_markers = MarkerArray()
        # If there are existing markers, delete them
        markers_to_delete = MarkerArray()
        if len(self.path_markers.markers) > 0:
            for marker in self.path_markers.markers:
                marker.action = Marker.DELETE
                markers_to_delete.markers.append(marker)
            self.path_marker_pub.publish(markers_to_delete)
        # Clear the path_markers
        self.path_markers = MarkerArray()
        line_strip_points = []
        # Create the waypoint markers
        for index, waypoint in enumerate(path):
            waypoint_marker = Marker()
            waypoint_marker.header.stamp = now
            waypoint_marker.header.frame_id = self.field_frame_id
            waypoint_marker.ns = "waypoints"
            waypoint_marker.id = index
            waypoint_marker.type = Marker.SPHERE
            if index == 0:
                waypoint_marker.type = Marker.CUBE
            waypoint_marker.action = Marker.ADD
            waypoint_marker.scale.x = 0.25
            waypoint_marker.scale.y = 0.25
            waypoint_marker.scale.z = 0.25
            point = Point(waypoint[0], waypoint[1], 0)
            waypoint_marker.pose.position = point
            # Store the point for the line_strip marker
            line_strip_points.append(point)
            # Color is based on path_status
            status = path_status[index]
            if status == 'not_visited':
                waypoint_marker.color = ColorRGBA(1,0,0,0.5)
            elif status == 'visiting':
                waypoint_marker.color = ColorRGBA(0,1,0,0.5)
            elif status == 'visited':
                waypoint_marker.color = ColorRGBA(0,0,1,0.5)
            else:
                rospy.err("Invalid path status.")
                waypoint_marker.color = ColorRGBA(1,1,1,0.5)
            # Put this waypoint Marker in the MarkerArray
            self.path_markers.markers.append(waypoint_marker)
        # Create the line_strip Marker which connects the waypoints
        line_strip = Marker()
        line_strip.header.stamp = now
        line_strip.header.frame_id = self.field_frame_id
        line_strip.ns = "lines"
        line_strip.id = 0
        line_strip.type = Marker.LINE_STRIP
        line_strip.action = Marker.ADD
        line_strip.scale.x = 0.1
        line_strip.color = ColorRGBA(0,0,1,0.5)
        line_strip.points = line_strip_points
        self.path_markers.markers.append(line_strip)
        # Publish the marker array
        self.path_marker_pub.publish(self.path_markers)

    def setup_path_following(self):
        """
        Sets up the node for following the planned path.
        """
        pass

    def step_path_following(self):
        """
        Steps the path following system, checking if new waypoints 
        should be sent, if a timeout has occurred, or if path following
        needs to be paused.
        """
        pass

if __name__ == '__main__':
    ppn = PathPlannerNode()
