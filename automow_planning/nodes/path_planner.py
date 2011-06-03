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
import shapely.geometry as sg

from nav_msgs.msg import Odometry, OccupancyGrid, GridCells
from geometry_msgs.msg import PolygonStamped, Point32, Polygon, Point, Twist
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from power_control_board.msg import CutterControl, PowerControl

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
            self.cutter_cooldown = rospy.get_param("~cutter_cooldown", 5.0)
            self.cost_threshold = rospy.get_param("~cost_threshold", 3)
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
        import copy
        new_points = copy.copy(self.points)
        # print "OldPoints", self.points
        for i in range(len(new_points)):
            new_points[i] = (new_points[i][0]*self.meters_per_cell,new_points[i][1]*self.meters_per_cell)
        # print "NewPoints",new_points
        self.sg_field_poly = sg.Polygon(new_points)
        
        # Occupancy Grid
        self.vis_pub = rospy.Publisher('coverage_map', GridCells)
        
        # Setup costmap and path planning
        self.setupCostmap()
        
        # Field polygons
        self.__polys = sg.MultiPolygon([sg.Polygon([(x,y),
                                                    (x+1,y),
                                                    (x+1,y+1),
                                                    (x,y+1)])
                                            for x in range(0, self.costmap.x_dim)
                                            for y in range(0, self.costmap.y_dim)])
        self.pick_furthest = rospy.get_param("~cutter_threshold", 0.5)

        # Connect ROS Topics
        self.current_position = None
        if not self.testing:
            rospy.Subscriber("ekf/odom", Odometry, self.odomCallback)
        
        self.left_cutter_cooldown = None
        self.left_cutter_state = False
        self.desired_left_cutter_state = False
        self.right_cutter_cooldown = None
        self.right_cutter_state = False
        self.desired_right_cutter_state = False
        rospy.Subscriber("/PowerControl", PowerControl, self.cutterCallback)
        self.pub_cutter = rospy.Publisher("/CutterControl", CutterControl)
        
        # Transformer
        self.tf_listener = tf.TransformListener()
        
        # Start spin thread
        threading.Thread(target=self.spin).start()
       
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

        return
        # Start Actionlib loop
        try:
            # Drive into the field first
            cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist)
            msg = Twist()
            msg.linear.x = 1.0
            msg.angular.z = 0.0
            
            cmd_vel_pub.publish(msg)
            
            rospy.sleep(3.0)
            
            msg.linear.x = 0.0
            
            cmd_vel_pub.publish(msg)
            
            rospy.sleep(1.0)
            
            self.desired_right_cutter_state = True
            self.desired_left_cutter_state = True
            self.setCutters()
            
            
            # Coverage
            self.performPathPlanning()
        finally:
            # Shutdown
            rospy.signal_shutdown("Done.")
    
    def cutterCallback(self, data):
        """docstring for cutterCallback"""
        self.left_cutter_state = data.LeftCutterStatus
        self.right_cutter_state = data.RightCutterStatus
    
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
        self.checkCutters()
        if self.current_position != (int(floor(x)), int(floor(y))):
            self.current_position = (int(floor(x)), int(floor(y)))
            self.costmap.setRobotPosition(y, x)
            rospy.loginfo("Setting robot position to %f, %f"%(x,y))
            self.publishVisualizations()
    
    def setCutters(self):
        """docstring for setCutters"""
        msg = CutterControl()
        msg.LeftControl = self.desired_left_cutter_state
        msg.RightControl = self.desired_right_cutter_state
        
        self.pub_cutter.publish(msg)
    
    def checkCutters(self):
        """docstring for checkCutters"""
        try:
            # self.tf_listener.waitForTransform("odom_combined", "left_cutter", )
            (left_trans, _ ) = self.tf_listener.lookupTransform('odom_combined',
                                                           'left_cutter',
                                                           rospy.Time(0))
            abs_left_cutter = sg.Point([left_trans[0], left_trans[1]]).buffer(0.3556/2.0)
            # print "abs_left_cutter: " + str(abs_left_cutter.bounds)
            area_ratio = self.sg_field_poly.intersection(abs_left_cutter).area/abs_left_cutter.area
            # print self.sg_field_poly.bounds
            print 'area:', area_ratio
            if area_ratio == 1.0:
                print 'it is inside'
                temp_state = False
            else:
                print 'it is outside'
                temp_state = True
            #if temp_state != self.desired_left_cutter_state:
            #    if self.left_cutter_cooldown == None or self.left_cutter_cooldown > rospy.Time.now()-rospy.Duration(self.cutter_cooldown):
            #        self.desired_left_cutter_state = temp_state
            #        self.left_cutter_cooldown = rospy.Time.now()
            
            (right_trans, _ ) = self.tf_listener.lookupTransform('odom_combined',
                                                           'right_cutter',
                                                           rospy.Time(0))
            abs_right_cutter = sg.Point([right_trans[0], right_trans[1]]).buffer(0.3556/2.0)
            
            area_ratio = self.sg_field_poly.intersection(abs_right_cutter).area/abs_right_cutter.area
            if area_ratio != 1.0:
                temp_state = False
            else:
                temp_state = True
            if temp_state != self.desired_right_cutter_state:
                if self.right_cutter_cooldown == None or self.right_cutter_cooldown > rospy.Time.now()-rospy.Duration(self.cutter_cooldown):
                    self.desired_right_cutter_state = temp_state
                    self.right_cutter_cooldown = rospy.Time.now()
            
            
            if self.desired_left_cutter_state != self.left_cutter_state or self.desired_right_cutter_state != self.right_cutter_state:
                self.setCutters()
            
            x = left_trans[0]
            x /= self.meters_per_cell
            x -= self.offset[0]
            y = left_trans[1]
            y /= self.meters_per_cell
            y = self.offset[1] - y
            left_trans = (x,y)

            left_cutter = sg.Point([left_trans[0], left_trans[1]]).buffer((0.3556/2.0)/self.meters_per_cell)
            
            if self.left_cutter_state:
                for cell in self.__polys:
                    if cell.intersects(left_cutter):
                        area_ratio = cell.intersection(left_cutter).area/cell.area
                        if area_ratio > self.pick_furthest:
                            self.costmap.consumeCell(left_trans[0], left_trans[1])
            
            x = right_trans[0]
            x /= self.meters_per_cell
            x -= self.offset[0]
            y = right_trans[1]
            y /= self.meters_per_cell
            y = self.offset[1] - y
            right_trans = (x,y)
            
            right_cutter = sg.Point([right_trans[0], right_trans[1]]).buffer((0.3556/2.0)/self.meters_per_cell)
            if self.right_cutter_state:
                for cell in self.__polys:
                    if cell.intersects(right_cutter):
                        area_ratio = cell.intersection(right_cutter).area/cell.area
                        if area_ratio > self.pick_furthest:
                            self.costmap.consumeCell(right_trans[0], right_trans[1])
        except (tf.LookupException, tf.ConnectivityException):
            return
    
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
        self.costmap = Costmap2D(self.cost_threshold)
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
        print 'publish'
        if not rospy.is_shutdown():
            self.poly_pub_timer = threading.Timer(1.0/self.field_shape_publish_rate, self.polyPublishHandler)
            self.poly_pub_timer.start()
        self.poly_msg.header.stamp = rospy.Time.now()
        self.poly_pub.publish(self.poly_msg)
    
    def publishVisualizations(self):
        """A function to publish an occupancy grid"""
        print self.costmap
        consumed_cells = self.costmap.getConsumedCell()
        
        msg = GridCells()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.field_frame_id
        msg.cell_width = self.meters_per_cell
        msg.cell_height = self.meters_per_cell
        
        cells = []
        
        for i in range(len(consumed_cells)):
            p = Point(consumed_cells[i][0], consumed_cells[i][1], 0.0)
            cells.append(p)
        
        msg.cells = cells
        
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
