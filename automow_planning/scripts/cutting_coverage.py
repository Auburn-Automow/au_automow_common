#!/usr/bin/env python

"""
This node keeps track of the area that has been covered while the
blades are on.  It publishes this information as a nav_msgs/GridCells msg to 
be visualized in rViz.
"""

import roslib; roslib.load_manifest('automow_planning')
import rospy
import tf

from threading import Thread, Lock

from geometry_msgs.msg import PolygonStamped, Point
from nav_msgs.msg import GridCells
from automow_node.msg import Automow_PCB
from automow_planning.maptools import image2array

import shapely.geometry as geo

class CuttingCoverage(object):
    """
    This is a ROS node that is 
    responsible for monitoring cutting coverage
    """
    def __init__(self):
        # Setup ROS node
        rospy.init_node('cutting_coverage')

        # ROS params
        self.left_cutter_frame_id = \
            rospy.get_param("~left_cutter_frame_id", "left_cutter")
        self.right_cutter_frame_id = \
            rospy.get_param("~right_cutter_frame_id", "right_cutter")
        self.cutter_radius = rospy.get_param("~cutter_radius", 0.3556/2.0)
        self.coverage_resolution = 1.0/rospy.get_param("~coverage_resolution", 0.1)
        publish_rate = rospy.Rate(rospy.get_param("~publish_rate", 1.0))
        self.check_rate = rospy.Rate(rospy.get_param("~check_rate", 20.0))

        # Setup publishers and subscribers
        rospy.Subscriber('/field/safety', PolygonStamped, self.field_callback)
        rospy.Subscriber('/automow_pcb/status', Automow_PCB, self.on_status)
        self.grid_cells_pub = rospy.Publisher('/cutter_coverage', GridCells, latch=True)
        self.listener = tf.TransformListener()

        # Setup initial variables
        self.field_shape = None
        self.field_frame_id = None
        self.left_cutter_state = False
        self.right_cutter_state = False
        self.cutter_pixels = None
        self.grid_cells_msg = GridCells()
        self.grid_cells_msg.cell_width = 1.0/self.coverage_resolution
        self.grid_cells_msg.cell_height = 1.0/self.coverage_resolution
        self.grid_cells_lock = Lock()

        # Wait for the field shape
        while self.field_shape == None:
            if rospy.is_shutdown(): return
            rospy.loginfo("Cutting Coverage: waiting on field shape")
            rospy.Rate(1.0).sleep()


        # Start the check thread
        check_thread = Thread(target=self.update_thread)
        check_thread.start()

        # Spin
        while not rospy.is_shutdown():
            # Publish periodically
            self.grid_cells_msg.header.stamp = rospy.Time.now()
            self.grid_cells_lock.acquire()
            self.grid_cells_pub.publish(self.grid_cells_msg)
            self.grid_cells_lock.release()
            publish_rate.sleep()
        # Join the other thread
        check_thread.join()

    def update_thread(self):
        """
        Thread that monitors the cutter positions and updates the grid cells accordingly.
        """
        exceptions = (tf.LookupException,
                          tf.ConnectivityException,
                          rospy.ServiceException)
        while not rospy.is_shutdown():
            self.check_rate.sleep()
            try:
                # Get the left cutter
                left_cutter = self.get_cutter_shape(self.field_frame_id,
                                               self.left_cutter_frame_id)
                # Get the right cutter
                right_cutter = self.get_cutter_shape(self.field_frame_id,
                                                self.right_cutter_frame_id)
                new_cells = self.update_coverage_map(left_cutter, right_cutter)
                self.grid_cells_lock.acquire()
                self.grid_cells_msg.cells.extend(new_cells)
                self.grid_cells_lock.release()
            except tf.ExtrapolationException as e:
                continue
            except exceptions as e:
                rospy.logwarn("Exception checking cutters: %s" % str(e))
                continue

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
        self.grid_cells_msg.header.frame_id = self.field_frame_id

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

    def update_coverage_map(self, left_cutter, right_cutter):
        """
        Takes the cutter shapes and then updates the coverage map.
        """
        new_cells = []
        # Add the left cutters
        if self.left_cutter_state: # If the left cutter is on
            points = self.get_raster_shape(left_cutter,
                                           self.coverage_resolution)
            for point in points:
                if point not in self.grid_cells_msg.cells:
                    new_cells.append(point)
        # Add the right cutters
        if self.right_cutter_state:
            points = self.get_raster_shape(right_cutter,
                                           self.coverage_resolution)
            for point in points:
                if point not in self.grid_cells_msg.cells:
                    new_cells.append(point)
        return new_cells

    def setup_raster_shape(self, cutter, resolution=100):
        """
        Creates the rasterized version of the cutter.
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
                    cut_pixels.append(point)
        self.cutter_pixels = cut_pixels

    def get_raster_shape(self, cutter, resolution=100):
        """
        Returns the locations, as Point's, of pixels that represent the cutter.
        """
        offset = (cutter.bounds[0], cutter.bounds[1])
        if self.cutter_pixels == None:
            self.setup_raster_shape(cutter, resolution)
        new_points = list(self.cutter_pixels)
        for point in new_points:
            point.x += int(offset[0]*self.coverage_resolution)/float(self.coverage_resolution)
            point.y += int(offset[1]*self.coverage_resolution)/float(self.coverage_resolution)
        return new_points

if __name__ == '__main__':
    ccn = CuttingCoverage()
