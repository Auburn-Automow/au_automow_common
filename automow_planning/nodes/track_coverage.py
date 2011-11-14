#!/usr/bin/env python

"""
Publish the coverage map.
"""

import roslib
roslib.load_manifest('automow_planning')

from nav_msgs.msg import GridCells

def main():
    rospy.init_node("coverage")

    cut_grass_pub = rospy.Publisher("cut_grass", GridCells)

    print "TODO(ashgti): implement the coverage map publisher."

if __name__ == '__main__':
    main()
