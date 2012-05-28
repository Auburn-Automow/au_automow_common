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
from automow_node.msg import Automow_PCB
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
        self.coverage_resolution = rospy.get_param("~coverage_resolution", 10)
        check_rate = rospy.Rate(rospy.get_param("~check_rate", 10.0))

        # Setup publishers and subscribers
        rospy.Subscriber('/field/boundry', PolygonStamped, self.field_callback)
        rospy.Subscriber('/automow_pcb/status', Automow_PCB, self.on_status)
        self.grid_cells_pub = rospy.Publisher('/cutter_coverage', GridCells)
        self.listener = tf.TransformListener()

        # Setup ROS service
        set_cutter_states = rospy.ServiceProxy('cutters', Cutters)

        # Setup initial variables
        self.field_shape = None
        self.field_frame_id = None
        self.left_cutter_state = False
        self.right_cutter_state = False
        self.grid_cells_msg = GridCells()

        # Set the initial cutter status to False
        set_cutter_states(False, False)

        # Spin
        while not rospy.is_shutdown():
            exceptions = (tf.LookupException,
                          tf.ConnectivityException,
                          rospy.ServiceException)
            try:
                # Update the proper cutter states
                left_state, right_state = self.check_cutters()
                set_cutter_states(left_state, right_state)
            except tf.ExtrapolationException as e:
                continue
            except exceptions as e:
                rospy.logwarn("Exception checking cutters: %s" % str(e))
                continue
            check_rate.sleep()

    def on_status(self, msg):
        """
        Gets called when a new automow_pcb state is published.

        Pulls the cutter states out and updates the internal state.
        """
        self.left_cutter_state = msg.cutter_1
        self.right_cutter_state = msg.cutter_2

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

    def update_coverage_map(self, left_cutter, right_cutter):
        """
        Takes the cutter shapes and then updates the coverage map.
        """
        # Update the GridCells msg
        self.grid_cells_msg.header.frame_id = self.field_frame_id
        self.grid_cells_msg.header.stamp = rospy.Time.now()
        self.grid_cells_msg.cell_width = 1.0/self.coverage_resolution
        self.grid_cells_msg.cell_height = 1.0/self.coverage_resolution
        # Add the left cutters
        if self.left_cutter_state: # If the left cutter is on
            points = self.get_raster_shape(left_cutter,
                                           self.coverage_resolution)
            for point in points:
                if point not in self.grid_cells_msg.cells:
                    self.grid_cells_msg.cells.append(point)
        # Add the right cutters
        if self.right_cutter_state:
            points = self.get_raster_shape(right_cutter,
                                           self.coverage_resolution)
            for point in points:
                if point not in self.grid_cells_msg.cells:
                    self.grid_cells_msg.cells.append(point)
        # Publish the updated msg
        self.grid_cells_pub.publish(self.grid_cells_msg)

    def get_raster_shape(self, cutter, resolution=100):
        """
        Returns the locations, as Point's, of pixels that represent the cutter.
        """
        # Get the shape in the image frame
        cutter_coords = []
        from math import ceil
        resolution = float(resolution)
        offset = (cutter.bounds[0], cutter.bounds[1])
        size = (cutter.bounds[2]-cutter.bounds[0])
        for coord in list(cutter.exterior.coords):
            # Convert from meters to image resolution
            new_coord = [coord[0]-offset[0], coord[1]-offset[1]]
            new_coord[0] *= resolution
            new_coord[1] *= resolution
            cutter_coords.append(tuple(new_coord))
        # Raster the polygon into an image using PIL
        import Image, ImageDraw
        dim = int(ceil(size*resolution))
        im = Image.new("L", (dim, dim), 0)
        draw = ImageDraw.Draw(im)
        draw.polygon(cutter_coords, fill=255, outline=255)
        del draw
        # Get the "cut" pixels
        cut_pixels = []
        for i, row in enumerate(image2array(im)):
            for j, element in enumerate(row):
                if element != 0:
                    point = Point(i/resolution, j/resolution, 0)
                    point.x += int(offset[0]*self.coverage_resolution)/float(self.coverage_resolution)
                    point.y += int(offset[1]*self.coverage_resolution)/float(self.coverage_resolution)
                    cut_pixels.append(point)
        return cut_pixels

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
        # Update the coverage map
        self.update_coverage_map(left_cutter, right_cutter)
        # Return the new states
        return (left_cutter_state, right_cutter_state)

if __name__ == '__main__':
    ccn = CutterControlNode()
