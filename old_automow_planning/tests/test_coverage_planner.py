#!/usr/bin/env python

"""
This will exercise the coverage planner and its abilities.
"""

import roslib; roslib.load_manifest('automow_planning')

# Python Libraries
import sys, os

# External Libraries
from shapely.geometry import Polygon

# ROS imports
from automow_planning import maptools
from maptools import RotationTransform, rotate_polygon_to, rotate_polygon_from

def test_1():
    pass

if __name__ == '__main__':
    test_1()

