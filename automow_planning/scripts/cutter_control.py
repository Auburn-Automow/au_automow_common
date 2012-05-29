#!/usr/bin/env python

"""
This ROS node takes the field polygon and the cutter 
positions and controls the cutting blades accordingly.

This node subscribes to the field_shape, which is published
as a geometry_msgs/PolygonStamped.  It also uses the tf provided
by the state publisher to figure out the position of the cutting
blades at any given time, cutting them on if the blades are in the field
and cutting them off otherwise.  It cuts them on and off using the 
service provided by the automow_pcb node.

This node uses the shapely library to figure out if the cutters
are mostly in the field or out of the field.

Additionally this node keeps track of the area that has been covered while the
blades are on.  It publishes this information as a nav_msgs/GridCells msg to 
be visualized in rViz.
"""

import roslib; roslib.load_manifest('automow_planning')
import rospy
import tf

from geometry_msgs.msg import PolygonStamped, Point
from nav_msgs.msg import GridCells
from automow_node.srv import Cutters
from automow_planning.maptools import image2array

import shapely.geometry as geo

class CutterControlNode(object):
    """
    This is a ROS node that is 
    responsible for controlling the cutters
    """
    def __init__(self):
        # Setup ROS node
        rospy.init_node('cutter_control')

        # ROS params
        self.left_cutter_frame_id = \
            rospy.get_param("~left_cutter_frame_id", "left_cutter")
        self.right_cutter_frame_id = \
            rospy.get_param("~right_cutter_frame_id", "right_cutter")
        self.cutter_radius = rospy.get_param("~cutter_radius", 0.3556/2.0)
        check_rate = rospy.Rate(rospy.get_param("~check_rate", 20.0))

        # Setup publishers and subscribers
        rospy.Subscriber('/field/boundry', PolygonStamped, self.field_callback)
        self.listener = tf.TransformListener()

        # Setup ROS service
        set_cutter_states = rospy.ServiceProxy('cutters', Cutters)

        # Setup initial variables
        self.field_shape = None
        self.field_frame_id = None
        self.left_cutter_state = False
        self.right_cutter_state = False

        # Set the initial cutter status to False
        set_cutter_states(False, False)
        self.current_left_state = False
        self.current_right_state = False

        # Spin
        while not rospy.is_shutdown():
            exceptions = (tf.LookupException,
                          tf.ConnectivityException,
                          rospy.ServiceException)
            try:
                # Update the proper cutter states
                left_state, right_state = self.check_cutters()
                if self.current_left_state != left_state or self.current_right_state != right_state:
                    set_cutter_states(left_state, right_state)
                self.current_left_state = left_state
                self.current_right_state = right_state
            except tf.ExtrapolationException as e:
                continue
            except exceptions as e:
                rospy.logwarn("Exception checking cutters: %s" % str(e))
                continue
            check_rate.sleep()

    def field_callback(self, msg):
        """
        Handles new field polygons, has to be called 
        at least once before cutters will be controlled.
        """
        # Convert the PolygonStamped into a shapely polygon
        temp_points = []
        for point in msg.polygon.points:
            temp_points.append( (float(point.x), float(point.y)) )
        self.field_shape = geo.Polygon(temp_points)
        self.field_frame_id = msg.header.frame_id

    def get_cutter_shape(self, field_frame, cutter_frame):
        """
        Returns a cutter shape given the field frame and the cutter frame.
        """
        # Get the position of the cutter in the field frame
        (translation, rotation) = self.listener.lookupTransform(field_frame,
                                                                cutter_frame,
                                                                rospy.Time(0))
        # Create a point at the frame location
        cutter = geo.Point([translation[0], translation[1]])
        # "buffer" the point into a circle using the cutter radius
        cutter = cutter.buffer(self.cutter_radius)
        return cutter

    def is_cutter_in_field(self, cutter):
        """
        Checks to see if the given cutter shape is in the field.
        """
        # Calculate the intersection of the cutter and field shapes
        intersection = self.field_shape.intersection(cutter)
        # Calculate the ratio of the intersection's area to the cutter's area
        area_ratio = intersection.area/cutter.area
        # If the ratio is not 1.0 then part of the cutter 
        # is outside of the field, so turn off the blades
        if area_ratio != 1.0:
            return False
        else:
            return True

    def check_cutters(self):
        """
        Waits for a transform from the field_frame to the 
        cutter frames and then checks to see if they lie 
        in the field polygon.
        """
        # Gate to prevent this when the field_shape has not been received
        if self.field_shape == None or self.field_frame_id == None:
            return (False, False)
        # Get the left cutter
        left_cutter = self.get_cutter_shape(self.field_frame_id,
                                       self.left_cutter_frame_id)
        # Get the right cutter
        right_cutter = self.get_cutter_shape(self.field_frame_id,
                                        self.right_cutter_frame_id)
        # Check to see if the left cutter is in the field polygon
        left_cutter_state = self.is_cutter_in_field(left_cutter)
        # Check to see if the right cutter is in the field polygon
        right_cutter_state = self.is_cutter_in_field(right_cutter)
        # Return the new states
        return (left_cutter_state, right_cutter_state)

if __name__ == '__main__':
    ccn = CutterControlNode()
