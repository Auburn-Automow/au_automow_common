#!/usr/bin/env python
import roslib; roslib.load_manifest('automow_planning')
import rospy
import actionlib
import tf

import os
from math import *
import Image, ImageDraw
import time
import threading

from maptools import *
from costmap import Costmap2D

from nav_msgs.msg import Odometry, OccupancyGrid, GridCells
from geometry_msgs.msg import PolygonStamped, Point32, Polygon
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from power_control_board.msg import CutterControl

class PathPlanner:
    def __init__(self, testing=False):
        self.testing = testing
        
        self.current_destination = -1
        
        if not self.testing:
            self.Init()
    
    def Init(self):
        """docstring for Init"""
        rospy.init_node('path_planner')
        
        if not self.testing:
            self.meters_per_cell = rospy.get_param("~meters_per_cell", 0.4)
            self.field_frame_id = rospy.get_param("~field_frame_id","odom_combined")
            self.goal_timeout = rospy.get_param("~goal_timeout", 15.0)
            self.pick_furthest = rospy.get_param("~pick_furthest", True)
        
        # Field File related stuff
        if not self.testing:
            self.file_name = rospy.get_param("~field_csv_file","/tmp/field.csv")
        if not os.path.exists(self.file_name):
            rospy.logerr("Specified field csv file does not exist: %s"%self.file_name)
            return
        rospy.loginfo("Using filed file: %s"%self.file_name)
        
        self.readFieldPolygon()
        if rospy.is_shutdown():
            return
        
        # Setup field polygon publisher
        if not self.testing:
            self.field_shape_publish_rate = rospy.get_param("~field_shape_publish_rate", 1.0)
        self.poly_pub = rospy.Publisher("field_shape", PolygonStamped)
        self.poly_pub_timer = threading.Timer(1.0/self.field_shape_publish_rate, self.polyPublishHandler)
        self.poly_pub_timer.start()
        
        # Occupancy Grid
        self.vis_pub = rospy.Publisher('coverage_map', GridCells)
        
        # Setup costmap and path planning
        self.setupCostmap()
        
        # Connect ROS Topics
        self.current_position = None
        if not self.testing:
            rospy.Subscriber("ekf/odom", Odometry, self.odomCallback)
        
        self.left_cutter_cooldown = None
        self.left_cutter_state = False
        self.right_cutter_cooldown = None
        self.right_cutter_state = False
        # self.cutter_publisher = rospy.Subscriber("/CutterControl", CutterControl)
        
        # Start spin thread
        threading.Thread(target=self.spin).start()
        
        # Start Actionlib loop
        try:
            self.performPathPlanning()
        finally:
            # Shutdown
            rospy.signal_shutdown("Done.")
    
    def cutterControlHandler(self):
        """docstring for cutterControlHandler"""
        rate = rospy.Rate(rospy.Duration(0.1))
        while not rospy.is_shutdown():
            pass
            rate.sleep()
    
    def point_inside_polygon(self,x,y,poly):
        """Originally from: http://www.ariel.com.au/a/python-point-int-poly.html"""
        n = len(poly)
        inside = False
        
        p1x,p1y = poly[0]
        for i in range(n+1):
            p2x,p2y = poly[i % n]
            if y > min(p1y,p2y):
                if y <= max(p1y,p2y):
                    if x <= max(p1x,p2x):
                        if p1y != p2y:
                            xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x,p1y = p2x,p2y
        
        return inside
    
    def odomCallback(self, data):
        """docstring for odomCallback"""
        x = data.pose.pose.position.x
        x /= self.meters_per_cell
        x -= self.offset[0]
        y = data.pose.pose.position.y
        y /= self.meters_per_cell
        y = self.offset[1] - y
        if self.current_position != (int(floor(x)), int(floor(y))):
            self.current_position = (int(floor(x)), int(floor(y)))
            self.costmap.setRobotPosition(y, x)
            rospy.loginfo("Setting robot position to %f, %f"%(x,y))
            self.publishVisualizations()
    
    def performPathPlanning(self):
        """docstring for performPathPlanning"""
        while self.costmap.robot_position == (-1,-1):
            rospy.loginfo("Waiting for robot position to be initialized.")
            rospy.sleep(3.0)
            if rospy.is_shutdown():
                return
        
        if not self.testing:
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            rospy.loginfo("Waiting for move_base...")
            client.wait_for_server()
        
        next_position = self.costmap.getNextPosition(self.pick_furthest)
        while next_position != None:
            if rospy.is_shutdown():
                break
            destination = MoveBaseGoal()
            destination.target_pose.header.frame_id = self.field_frame_id
            destination.target_pose.header.stamp = rospy.Time.now()
            
            (x,y) = next_position
            
            x += self.offset[0]
            x *= self.meters_per_cell
            y = self.offset[1] - y
            y *= self.meters_per_cell
            
            next_position = (x,y)
            
            destination.target_pose.pose.position.x = next_position[0]
            destination.target_pose.pose.position.y = next_position[1]
            quat = tf.transformations.quaternion_from_euler(0, 0, radians(90))
            destination.target_pose.pose.orientation.x = quat[0]
            destination.target_pose.pose.orientation.y = quat[1]
            destination.target_pose.pose.orientation.z = quat[2]
            destination.target_pose.pose.orientation.w = quat[3]
            
            rospy.loginfo("Sending goal %f, %f to move_base..."%next_position)
            
            if not self.testing:
                client.send_goal(destination)
            else:
                self.current_destination = destination
            
            if client.wait_for_result(rospy.Duration(self.goal_timeout)):
                rospy.loginfo("Goal reached successfully!")
            else:
                rospy.logwarn("Goal aborted!")
            
            next_position = self.costmap.getNextPosition(self.pick_furthest)
        self.current_destination = None
    
    def setupCostmap(self):
        """docstring for setupCostmap"""
        # Draw field polygon on the soon to be map image
        self.map_img = Image.new("L", self.size, 255)
        
        # Transform map coordinate into image frame
        img_points = []
        for x in range(len(self.points)):
            img_points.append(
                              (
                               floor(self.points[x][0])-self.offset[0],
                               self.offset[1]-floor(self.points[x][1])
                              )
                             )
        print self.points
        print img_points
        
        # Draw the polygon
        draw = ImageDraw.Draw(self.map_img)
        draw.polygon(img_points, fill=0)
        
        # Uncomment for "flowerbed" in the middle of the field
        # rad = 1
        # draw.ellipse((self.size[0]/4-rad,self.size[1]/4-rad)+(self.size[0]/4+rad,self.size[1]/4+rad), fill=11)
        
        # TODO: add obstacles from the laser range finder at this point
        del draw
        
        # Create and initialize the costmap
        self.costmap = Costmap2D(1)
        self.costmap.setData(image2array(self.map_img))
        
        # Plan coverage
        tick = time.time()
        self.costmap.planCoverage()
        tock = (time.time()-tick)*1000.0
        # print self.costmap
        rospy.loginfo("Map coverage planning completed in %f milliseconds."%tock)
    
    def spin(self):
        """docstring for spin"""
        rospy.spin()
    
    def polyPublishHandler(self):
        """docstring for polyPublishHandler"""
        if not rospy.is_shutdown():
            self.poly_pub_timer = threading.Timer(1.0/self.field_shape_publish_rate, self.polyPublishHandler)
            self.poly_pub_timer.start()
        self.poly_msg.header.stamp = rospy.Time.now()
        self.poly_pub.publish(self.poly_msg)
    
    def publishVisualizations(self):
        """A function to publish an occupancy grid"""
        consumed_cells = self.costmap.getConsumedCells()
        
        msg = GridCells()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.field_frame_id
        msg.cell_width = self.meters_per_cell
        msg.cell_height = self.meters_per_cell
        
        msg.cells.resize(len(consumed_cells))
        
        for i in range(len(consumed_cells)):
            msg.cells[i].x = consumed_cells[i][0]
            msg.cells[i].y = consumed_cells[i][1]
            msg.cells[i].z = 0.0
        
        self.vis_pub.publish(msg)
    
    def readFieldPolygon(self):
        """docstring for readFieldPolygon"""
        f = open(self.file_name, 'r')
        
        lines = f.read()
        lines = lines.split("\n")
        
        self.points = []
        self.point32s = []
        for line in lines:
            line = line.strip()
            if line == "":
                continue
            split_stuff = line.split(",")
            if len(split_stuff) == 3:
                (east,north,fix) = split_stuff
                self.points.append((float(east)/self.meters_per_cell,float(north)/self.meters_per_cell))
                self.point32s.append(Point32(float(east),float(north),0.0))
            else:
                rospy.logerr("Error unpacking field csv...")
                rospy.signal_shutdown("Error unpacking field csv...")
                return
        
        xs,ys = zip(*self.points)
        offset = self.offset = (int(ceil(min(xs)))-4, int(floor(max(ys)))+4) # (east, north)
        size = self.size = (int(ceil(max(xs)) - floor(min(xs)))+8, int(ceil(max(ys)) - floor(min(ys)))+8)
        rospy.loginfo("Map Offset East: %f cells (%f meters)"%(offset[0],offset[0]*self.meters_per_cell))
        rospy.loginfo("Map Offset North: %f cells (%f meters)"%(offset[1],offset[1]*self.meters_per_cell))
        rospy.loginfo("Map Size East: %f cells (%f meters)"%(size[0],size[0]*self.meters_per_cell))
        rospy.loginfo("Map Size North: %f cells (%f meters)"%(size[1],size[1]*self.meters_per_cell))
        
        self.poly_msg = PolygonStamped(Header(),Polygon(self.point32s))
        self.poly_msg.header.frame_id = self.field_frame_id
    

if __name__ == '__main__':
    pp = PathPlanner()
