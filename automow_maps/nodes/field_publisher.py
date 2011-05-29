#!/usr/bin/env python
import roslib; roslib.load_manifest('automow_maps')
import rospy

from geometry_msgs.msg import PolygonStamped, Point32, Polygon
from std_msgs.msg import Header

points = None
poly = None

def main():
    global poly, points
    """docstring for main"""
    rospy.init_node('field_publisher')
    
    file_name = rospy.get_param("~field_csv_file","/tmp/field.csv")
    
    f = open(file_name, 'r')
    
    lines = f.read()
    lines = lines.split("\n")
    
    points = []
    for line in lines:
        line = line.strip()
        if line == "":
            continue
        split_stuff = line.split(",")
        if len(split_stuff) == 3:
            (x,y,fix) = split_stuff
            points.append(Point32(float(x),float(y),0.0))
        else:
            rospy.logwarn("Error unpacking: %s" % line)
    
    poly = PolygonStamped(Header(),Polygon(points))
    poly.header.stamp = rospy.Time.now()
    poly.header.frame_id="odom_combined"
    
    pub = rospy.Publisher("field_shape", PolygonStamped)
    
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        # rospy.spinOnce()
        pub.publish(poly)
        rate.sleep()
    

if __name__ == '__main__':
    main()
