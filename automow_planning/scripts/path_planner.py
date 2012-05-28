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
from tf.transformations import quaternion_from_euler as qfe
from actionlib import SimpleActionClient

import numpy as np
from math import radians

from geometry_msgs.msg import PolygonStamped, Point, PoseStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path, Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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
        rospy.Subscriber('/field/cut_area', PolygonStamped, self.field_callback)
        self.path_marker_pub = rospy.Publisher('visualization_marker',
                                               MarkerArray,
                                               latch=True)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Setup initial variables
        self.field_shape = None
        self.field_frame_id = None
        self.path = None
        self.path_status = None
        self.path_markers = None
        self.start_path_following = True
        self.robot_pose = None
        self.goal_state = None
        self.current_destination = None
        self.testing = False
        self.current_distance = None
        self.previous_destination = None

        # Spin until shutdown or we are ready for path following
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and not self.start_path_following:
            rate.sleep()
        # If shutdown, return now
        if rospy.is_shutdown():
            return
        # Setup path following
        self.setup_path_following()
        # Iterate on path following
        while not rospy.is_shutdown():
            if not self.step_path_following():
                break

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

    def odom_callback(self, msg):
        """
        Watches for the robot's Odometry data, which is used in the path
        planning as the initial robot position.
        """
        self.robot_pose = msg

    def plan_path(self, field_polygon, origin=None):
        """
        This is called after a field polygon has been received.

        This uses the automow_planning.coverage module to plan a
        coverage path using the field geometry.  The path consists of
        a series of waypoints.
        """
        # Get the rotation to align with the longest edge of the polygon
        from automow_planning.maptools import rotation_tf_from_longest_edge, RotationTransform
        rotation = rotation_tf_from_longest_edge(field_polygon)
        rotation = RotationTransform(rotation.w + 90)
        # Rotate the field polygon
        from automow_planning.maptools import rotate_polygon_to
        transformed_field_polygon = rotate_polygon_to(field_polygon, rotation)
        # Decompose the rotated field into a series of waypoints
        from automow_planning.coverage import decompose
        print origin
        if origin is not None:
            point_mat = np.mat([[origin[0], origin[1], 0]], dtype='float64').transpose()
            origin = rotation.irm * point_mat
            origin = (origin[0,0], origin[1,0])
        transformed_path = decompose(transformed_field_polygon,
                                     origin=(origin[0], origin[1]),
                                     width=self.cut_spacing)
        # Rotate the transformed path back into the source frame
        from automow_planning.maptools import rotate_from
        self.path = rotate_from(np.array(transformed_path), rotation)
        # Calculate headings and extend the waypoints with them
        self.path = self.calculate_headings(self.path)
        # Set the path_status to 'not_visited'
        self.path_status = []
        for waypoint in self.path:
            self.path_status.append('not_visited')
        # Visualize the data
        self.visualize_path(self.path, self.path_status)

    def calculate_headings(self, path):
        """
        Calculates the headings between paths and adds them to the waypoints.
        """
        new_path = []
        for index, waypoint in enumerate(path):
            new_path.append(list(path[index]))
            # If the end, copy the previous heading
            if index == 0:
                new_path[index].append(0)
                continue
            # Calculate the angle between this waypoint and the next
            dx = path[index][0] - path[index-1][0]
            dy = path[index][1] - path[index-1][1]
            from math import atan2, pi
            heading = atan2(dy, dx)
            new_path[index].append(heading)
        return new_path

    def visualize_path(self, path, path_status=None):
        """
        Convenience function, calls both visualize_path{as_path, as_marker}
        """
        # TODO: finish this (path as path viz)
        # self.visualize_path_as_path(path, path_status)
        self.visualize_path_as_marker(path, path_status)

    def visualize_path_as_path(self, path, path_status=None):
        """
        Publishes a nav_msgs/Path msg containing the planned path.

        If path_status is not None then the only waypoints put in the
        nav_msgs/Path msg will be ones that are 'not_visited' or 'visiting'.
        """
        now = rospy.Time.now()
        msg = Path()
        msg.header.stamp = now
        msg.header.frame_id = self.field_frame_id
        for index, waypoint in enumerate(path):
            # Only put 'not_visited', 'visiting', and the most recent 'visited'
            # in the path msg
            if path_status != None: # If not set, ignore
                if path_status[index] == 'visited': # if this one is visited
                    try:
                        # if the next one is visited too
                        if path_status[index+1] == 'visited':
                            # Then continue, because this one doesn't belong
                            # in the path msg
                            continue
                    except KeyError as e: # incase index+1 is too big
                        pass
            # Otherwise if belongs, put it in
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now
            pose_msg.header.frame_id = self.field_frame_id
            pose_msg.pose.position.x = waypoint[0]
            pose_msg.pose.position.y = waypoint[1]
            msg.poses.append(pose_msg)

    def visualize_path_as_marker(self, path, path_status):
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
        # # If there are existing markers, delete them
        # markers_to_delete = MarkerArray()
        # if len(self.path_markers.markers) > 0:
        #     for marker in self.path_markers.markers:
        #         marker.action = Marker.DELETE
        #         markers_to_delete.markers.append(marker)
        #     self.path_marker_pub.publish(markers_to_delete)
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
            waypoint_marker.type = Marker.ARROW
            if index == 0:
                waypoint_marker.type = Marker.CUBE
            waypoint_marker.action = Marker.MODIFY
            waypoint_marker.scale.x = 1
            waypoint_marker.scale.y = 1
            waypoint_marker.scale.z = 0.25
            point = Point(waypoint[0], waypoint[1], 0)
            waypoint_marker.pose.position = point
            # Store the point for the line_strip marker
            line_strip_points.append(point)
            # Set the heading of the ARROW
            quat = qfe(0, 0, waypoint[2])
            waypoint_marker.pose.orientation.x = quat[0]
            waypoint_marker.pose.orientation.y = quat[1]
            waypoint_marker.pose.orientation.z = quat[2]
            waypoint_marker.pose.orientation.w = quat[3]
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

        Will wait until the initial robot pose is set and until
        the move_base actionlib service is available.
        """
        # Create the actionlib service
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        connected_to_move_base = False
        dur = rospy.Duration(1.0)
        # If testing prime the robot_pose
        if self.testing:
            self.robot_pose = Odometry()
            self.robot_pose.pose.pose.position.x = 0
            self.robot_pose.pose.pose.position.y = 0
        # Wait for the field shape
        while self.field_shape == None:
            # Check to make sure ROS is ok still
            if rospy.is_shutdown(): return
            # Print message about the waiting
            msg = "Path Planner: waiting on the field shape."
            rospy.loginfo(msg)
            rospy.Rate(1.0).sleep()
        # Wait for the robot position
        while self.robot_pose == None:
            # Check to make sure ROS is ok still
            if rospy.is_shutdown(): return
            # Print message about the waiting
            msg = "Path Planner: waiting on initial robot pose."
            rospy.loginfo(msg)
            rospy.Rate(1.0).sleep()
        # Now we should plan a path using the robot's initial pose
        origin = (self.robot_pose.pose.pose.position.x,
                  self.robot_pose.pose.pose.position.y)
        self.plan_path(self.field_shape, origin)
        rospy.loginfo("Path Planner: path planning complete.")
        # Now wait for move_base
        while connected_to_move_base:
            # Wait for the server for a while
            connected_to_move_base = self.move_base_client.wait_for_server(dur)
            # Check to make sure ROS is ok still
            if rospy.is_shutdown(): return
            # Update the user on the status of this process
            msg = "Path Planner: waiting on move_base."
            rospy.loginfo(msg)
        # Now we are ready to start feeding move_base waypoints
        return

    def get_next_waypoint_index(self):
        """
        Figures out what the index of the next waypoint is.

        Iterates through the path_status's and finds the visiting one,
        or the next not_visited one if not visiting on exists.
        """
        for index, status in enumerate(self.path_status):
            if status == 'visited':
                continue
            if status == 'visiting':
                return index
            if status == 'not_visited':
                return index
        # If I get here then there are no not_visited and we are done.
        return None

    def distance(self, p1, p2):
        from math import sqrt
        dx = p2.target_pose.pose.position.x - p1.target_pose.pose.position.x
        dy = p2.target_pose.pose.position.y - p1.target_pose.pose.position.y
        return sqrt(dx**2 + dy**2)

    def step_path_following(self):
        """
        Steps the path following system, checking if new waypoints
        should be sent, if a timeout has occurred, or if path following
        needs to be paused.
        """
        # Visualize the data
        self.visualize_path(self.path, self.path_status)
        # Get the next (or current) waypoint
        current_waypoint_index = self.get_next_waypoint_index()
        # If the index is None, then we are done path planning
        if current_waypoint_index == None:
            rospy.loginfo("Path Planner: Done.")
            return False
        if current_waypoint_index == 0:
            self.path_status[current_waypoint_index] = 'visited'
        # Get the waypoint and status
        current_waypoint = self.path[current_waypoint_index]
        current_waypoint_status = self.path_status[current_waypoint_index]
        # If the status is visited
        if current_waypoint_status == 'visited':
            # This shouldn't happen...
            return True
        # If the status is not_visited then we need to push the goal
        if current_waypoint_status == 'not_visited':
            # Cancel any current goals
            self.move_base_client.cancel_all_goals()
            # Set it to visiting
            self.path_status[current_waypoint_index] = 'visiting'
            # Push the goal to the actionlib server
            destination = MoveBaseGoal()
            destination.target_pose.header.frame_id = self.field_frame_id
            destination.target_pose.header.stamp = rospy.Time.now()
            # Set the target location
            destination.target_pose.pose.position.x = current_waypoint[0]
            destination.target_pose.pose.position.y = current_waypoint[1]
            # Calculate the distance
            if self.previous_destination == None:
                self.current_distance = 5.0
            else:
                self.current_distance = self.distance(self.previous_destination, destination)
            # Set the heading
            quat = qfe(0, 0, current_waypoint[2])
            destination.target_pose.pose.orientation.x = quat[0]
            destination.target_pose.pose.orientation.y = quat[1]
            destination.target_pose.pose.orientation.z = quat[2]
            destination.target_pose.pose.orientation.w = quat[3]
            # Send the desired destination to the actionlib server
            rospy.loginfo("Sending waypoint (%f, %f)@%f" % tuple(current_waypoint))
            self.current_destination = destination
            self.move_base_client.send_goal(destination)
            self.previous_destination = destination
        # If the status is visiting, then we just need to monitor the status
        if current_waypoint_status == 'visiting':
            temp_state = self.move_base_client.get_goal_status_text()
            # Figure out the msg and action based on the state
            msg = "Current waypoint (%f, %f)@%f is " % tuple(current_waypoint)
            msg += temp_state
            if temp_state in ['ABORTED', 'SUCCEEDED']:
                self.path_status[current_waypoint_index] = 'visited'
            else:
                duration = rospy.Duration(1.0)
                from math import floor
                count = 0
                while not self.move_base_client.wait_for_result(duration) and count != floor(self.current_distance*20):
                    if rospy.is_shutdown(): return False
                    count += 1
                if count == floor(self.current_distance*20):
                    rospy.logwarn("Path Planner: move_base goal timeout occurred")
                self.path_status[current_waypoint_index] = 'visited'
        return True

if __name__ == '__main__':
    ppn = PathPlannerNode()
