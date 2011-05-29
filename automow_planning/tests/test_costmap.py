import sys
sys.path.append("../src")
from costmap import Costmap2D
from maptools import *

from math import *

import numpy as np

def test_costmap(example_map):
    my_costmap = Costmap2D()
    my_costmap.setData(example_map)
    import time
    tock = time.time()
    finished = my_costmap.planCoverage()
    tick = time.time()
    print my_costmap
    if not finished:
        print "Failed to cover the field in %f iterations."%my_costmap.COVERAGE_ITERATION_LIMIT
    print "Coverage Expansion Execution Time: %f milliseconds" % ((tick-tock)*1000.0)
    
    return my_costmap

def generateMapFromCSV(file_name, meters_per_cell):
    """docstring for generateMapFromCSV"""
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
            points.append((float(x)/meters_per_cell,-1*float(y)/meters_per_cell))
        else:
            print "Error unpacking csv..."
    xs,ys = zip(*points)
    offset = (int(floor(min(xs)))-1, int(floor(min(ys)))-1)
    size = (int(ceil(max(xs)) - floor(min(xs)))+2, int(ceil(max(ys)) - floor(min(ys)))+2)
    print "Map Offset X:", offset[0], "cells (",offset[0]*meters_per_cell, "meters )"
    print "Map Offset Y:", offset[1], "cells (",offset[1]*meters_per_cell, "meters )"
    print "Map Size X:", size[0], "cells (",size[0]*meters_per_cell, "meters )"
    print "Map Size Y:", size[1], "cells (",size[1]*meters_per_cell, "meters )"
    
    import Image, ImageDraw
    
    im = Image.new("L", size, 255)
    
    img_points = []
    for x in range(len(points)):
        img_points.append((floor(points[x][0])-offset[0], floor(points[x][1])-offset[1]))
    
    draw = ImageDraw.Draw(im)
    draw.polygon(img_points, fill=0)
    rad = 1
    draw.ellipse((size[0]/2-rad,size[1]/2-rad)+(size[0]/2+rad,size[1]/2+rad), fill=255)
    del draw
    
    return image2array(im)

def simulate_path_consumption(my_costmap, meters_per_cell):
    """docstring for simulate_path_consumption"""
    my_costmap.setRobotPosition(1,1)
    print my_costmap
    count = 0
    while True:
        my_costmap.setRobotPosition(*my_costmap.getNextPosition())
        print my_costmap
        count += 1
        stuff = raw_input()
        if stuff == 'e':
            break


if __name__ == '__main__':
    mpc = 0.8
    my_costmap = test_costmap(generateMapFromCSV("survey.csv", mpc))
    simulate_path_consumption(my_costmap, mpc)

























