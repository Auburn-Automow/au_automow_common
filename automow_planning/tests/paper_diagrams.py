#!/usr/bin/env python

"""
This modules contains functions for generating the plots and diagrams 
for the final paper.
"""

import sys, os

import roslib; roslib.load_manifest('automow_planning')

from automow_planning.maptools import *
from automow_planning.coverage import *

def get_competition_field():
    """returns the competition field"""
    import csv
    pkg_dir = roslib.packages.get_pkg_dir('automow_maps')
    data_file = os.path.join(pkg_dir,"maps","competition_field.csv")
    data = csv.reader(open(data_file,"r"))
    points = []
    for row in data:
        if len(row) == 3:
            points.append((float(row[0]),float(row[1])))
    # Normalize
    x,y = zip(*points)
    min_x = min(x)-1
    min_y = min(y)-1
    for i,point in enumerate(points):
        points[i] = (point[0]-min_x, point[1]-min_y)
    return points

def field_with_obstacle():
    """returns a field, ext, and an obstacle in it, inter"""
    ext = [(0, 0), (2, 5), (0, 11), (10, 11), (15, 5), (10, 0), (0, 0)]
    inter = [(3, 3), (3, 7), (7, 7), (7,3), (3, 3)]
    return ext, inter

def figure1():
    """Shows the different angles for planning"""
    def render_subplot(rotation, ax):
        # ext = [(1, 1), (-3, 11), (4, 16), (10, 10)]
        # ext = [(1, 1), (1, 11), (21, 11), (21, 1)]
        ext = get_competition_field()
        polygon = Polygon(ext)
        polygon_points = np.array(polygon.exterior)
        
        rt = RotationTransform(rotation)
        
        tf_polygon = rotate_polygon_to(polygon, rt)
        
        origin = rotate_to(np.array([(1,1)]),rt).tolist()
        
        from time import time; start = time()
        result = decompose(tf_polygon, origin, width=0.5)
        print "Decomposition Time:", time()-start
        
        tf_result = rotate_from(np.array(result), rt)
        
        ll = LineString(result)
        ll_tf = LineString(tf_result)
        
        plot_polygon(polygon, ax, color='red')
        plot_line(ll_tf, ax)
        ax.axvline(x=0, color='black')
        ax.axhline(y=0, color='black')
        zoom_extents(ax, [polygon])
    
    from pylab import plot, figure
    fig = figure(1, dpi=90)
    ax = fig.add_subplot(1,3,1)
    bx = fig.add_subplot(1,3,2,sharex=ax,sharey=ax)
    cx = fig.add_subplot(1,3,3,sharex=ax,sharey=ax)
    
    render_subplot(0,ax)
    render_subplot(90,bx)
    render_subplot(37.5,cx)
    
    import pylab
    pylab.setp(bx.get_yticklabels(), visible=False)
    pylab.setp(cx.get_yticklabels(), visible=False)
    
    bx.set_xlabel("Planned Coverage at different angles 0, 90, and 37.5 respectively", fontsize=14, horizontalalignment='center')
    
    pylab.show()

if __name__ == '__main__':
    figure1()