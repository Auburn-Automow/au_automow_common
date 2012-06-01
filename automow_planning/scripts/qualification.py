#!/usr/bin/env python

"""
This ROS node is responsible for executing a qualification run.
"""

import roslib; roslib.load_manifest('automow_planning')
import rospy
import tf
from tf.transformations import quaternion_from_euler as qfe
from actionlib import SimpleActionClient

import numpy as np
from math import radians

from geometry_msgs.msg import PolygonStamped, Point, PoseStamped, PointStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path, Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from automow_node.srv import Cutters

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
        self.box_size = rospy.get_param("~box_size", 5)

        rospy.Subscriber('/ekf/odom', Odometry, self.odom_callback)
        self.listener = tf.TransformListener()
        set_cutter_states = rospy.ServiceProxy('cutters', Cutters)
        set_cutter_states(False, False)

        # Setup initial variables
        self.robot_pose = None

        # Setup path following
        self.setup_path_following()
        set_cutter_states(True, True)
        self.do_qualification()
        set_cutter_states(False, False)

    def do_qualification(self):
        """
        Executes a qualification run.
        """
        for waypoint in self.path:
            if rospy.is_shutdown(): return
            # Cancel any current goals
            self.move_base_client.cancel_all_goals()
            # Push the goal to the actionlib server
            destination = MoveBaseGoal()
            destination.target_pose.header.frame_id = "odom"
            destination.target_pose.header.stamp = rospy.Time.now()
            # Set the target location
            destination.target_pose.pose.position.x = waypoint[0]
            destination.target_pose.pose.position.y = waypoint[1]
            # Set the heading
            quat = qfe(0, 0, waypoint[2])
            destination.target_pose.pose.orientation.x = quat[0]
            destination.target_pose.pose.orientation.y = quat[1]
            destination.target_pose.pose.orientation.z = quat[2]
            destination.target_pose.pose.orientation.w = quat[3]
            # Send the desired destination to the actionlib server
            rospy.loginfo("Sending waypoint (%f, %f)@%f" % tuple(waypoint))
            self.move_base_client.send_goal(destination)
            # Wait for the goal to finish
            duration = rospy.Duration(1.0)
            for x in range(60):
                self.move_base_client.wait_for_result(duration)
                if rospy.is_shutdown(): return

    def odom_callback(self, msg):
        """
        Watches for the robot's Odometry data, which is used in the path
        planning as the initial robot position.
        """
        self.robot_pose = msg

    def calculate_headings(self, path):
        """
        Calculates the headings between paths and adds them to the waypoints.
        """
        new_path = []
        for index, waypoint in enumerate(path):
            new_path.append(list(path[index]))
            # If the end, copy the previous heading
            if index == len(path)-1:
                new_path[index].append(new_path[index-1][2])
                continue
            # Calculate the angle between this waypoint and the next
            dx = path[index+1][0] - path[index][0]
            dy = path[index+1][1] - path[index][1]
            from math import atan2, pi
            heading = atan2(dy, dx)
            new_path[index].append(heading)
        return new_path

    def plan_path(self):
        """
        Uses the robot's current position to plan a path.
        """
        self.listener.waitForTransform("odom", "base_footprint", rospy.Time(), rospy.Duration(10.0))
        path = []
        indecies = [(0,0), (5,0), (5,5), (0,5), (0,0)]
        now = rospy.Time.now()
        for i, j in indecies:
            ps = PointStamped()
            ps.header.frame_id = "base_footprint"
            ps.header.stamp = now
            ps.point.x = i
            ps.point.y = j
            while True:
                try:
                    tf_ps = self.listener.transformPoint("odom", ps)
                    break
                except tf.ExtrapolationException as e:
                    continue
            path.append([tf_ps.point.x, tf_ps.point.y])
        path = self.calculate_headings(path)
        path.append(path[0])
        self.path = path

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
        # Wait for the robot position
        while self.robot_pose == None:
            # Check to make sure ROS is ok still
            if rospy.is_shutdown(): return
            # Print message about the waiting
            msg = "Qualification: waiting on initial robot pose."
            rospy.loginfo(msg)
            rospy.Rate(1.0).sleep()
        # Now we should plan a path
        self.plan_path()
        rospy.loginfo("Qualification: path planning complete.")
        # Now wait for move_base
        while not connected_to_move_base:
            # Wait for the server for a while
            connected_to_move_base = self.move_base_client.wait_for_server(dur)
            # Check to make sure ROS is ok still
            if rospy.is_shutdown(): return
            # Update the user on the status of this process
            msg = "Qualification: waiting on move_base."
            rospy.loginfo(msg)
        # Now we are ready to start feeding move_base waypoints
        return

if __name__ == '__main__':
    ppn = PathPlannerNode()
