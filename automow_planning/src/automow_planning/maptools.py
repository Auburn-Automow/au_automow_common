#!/usr/bin/env python

"""
Contains utilities for working with maps and field shapes.
"""

import itertools
import numpy as np
from PIL import Image
from shapely.geometry import Polygon, LineString
from matplotlib.collections import LineCollection
from math import cos, sin, sqrt, atan, degrees, radians

##### Numpy-PIL Functions #####
def image2array(im):
    if im.mode not in ("L", "F"):
        raise ValueError, "can only convert single-layer images"
    if im.mode == "L":
        a = np.fromstring(im.tostring(), dtype=np.uint8)
    else:
        a = np.fromstring(im.tostring(), dtype=np.float32)
    a.shape = im.size[1], im.size[0]
    return a

def array2image(a):
    if a.typecode() == np.uint8:
        mode = "L"
    elif a.typecode() == np.float32:
        mode = "F"
    else:
        raise ValueError, "unsupported image mode %s"%(a.typecode())
    return Image.fromstring(mode, (a.shape[1], a.shape[0]), a.tostring())


##### Numpy-Shapely Functions #####
def ndarray2polygon(points):
    if type(points) != np.ndarray:
        raise TypeError("ndarray2polygon: takes an numpy.ndarray")
    new_tuples = []
    for point in points:
        new_tuples.append((float(point[0]),float(point[1])))
    return Polygon(new_tuples)


##### Rotation Transformations #####
class RotationTransform:
    """Represents a rotational transform"""
    def __init__(self, angle):
        self.angle = angle
        self.w = radians(self.angle)
        self.irm = np.mat([[cos(self.w), -sin(self.w), 0.0],
                           [sin(self.w), cos(self.w),  0.0],
                           [0.0,         0.0,          1.0]])
    

def rotation_tf_from_longest_edge(polygon):
    """Returns a rotation tf for the longest edge of the given polygon"""
    max_distance = None
    max_points = [(None, None),(None, None)]
    # Find the longest edge and the points that make it
    xs, ys = polygon.exterior.xy
    points = zip(xs, ys)
    for i in range(len(points)):
        if i == len(points)-1:
            pair = (points[i],points[0])
        else:
            pair = (points[i],points[i+1])
        distance = sqrt((pair[1][0]-pair[0][0])**2+(pair[1][1]-pair[0][1])**2)
        if max_distance == None or max_distance < distance:
            max_distance = distance
            max_points = pair
    # Calculate the angle and return the rotation tf
    dy = float(max_points[0][1] - max_points[1][1])
    dx = float(max_points[0][0] - max_points[1][0])
    return RotationTransform((degrees(atan(dy/dx))))

def rotate_polygon_to(polygon, rotation_transform):
    """Takes a polygon and a rotation, returns a rotated polygon"""
    points = np.array(polygon.exterior)
    tf_points = rotate_to(points, rotation_transform)
    return ndarray2polygon(tf_points)

def rotate_polygon_from(polygon, rotation_transform):
    """Takes a polygon and a rotation, returns an inverse rotated polygon"""
    points = np.array(polygon.exterior)
    tf_points = rotate_from(points, rotation_transform)
    return ndarray2polygon(tf_points)

def rotate_to(points, rotation_transform):
    """Rotates an ndarray of given points(x,y) to a given rotation"""
    if type(points) != np.ndarray:
        raise TypeError("rotate_to: takes an numpy.ndarray")
    new_points = []
    for point in points:
        point_mat = np.mat([[point[0]],[point[1]],[0]], dtype='float64')
        new_point = rotation_transform.irm * point_mat
        new_points.append(np.array(new_point[:-1].T, dtype='float64'))
    return np.squeeze(np.array(new_points, dtype='float64'))

def rotate_from(points, rotation_transform):
    """Rotate an ndarray of given points(x,y) from a given rotation"""
    if type(points) != np.ndarray:
        raise TypeError("rotate_from: takes an numpy.ndarray")
    new_points = []
    for point in points:
        point_mat = np.mat([[point[0]],[point[1]],[0]], dtype='float64')
        new_point = rotation_transform.irm.I * point_mat
        new_points.append(np.array(new_point[:-1].T, dtype='float64'))
    return np.squeeze(np.array(new_points, dtype='float64'))


##### Plotting Tools #####

def zoom_extents(ax, polygons, buff=1.0):
    """Sets the axis to view all polygons given"""
    min_x = None
    max_x = None
    min_y = None
    max_y = None
    for polygon in polygons:
        bounds = polygon.bounds
        if min_x == None or bounds[0] < min_x:
            min_x = bounds[0]
        if min_y == None or bounds[1] < min_y:
            min_y = bounds[1]
        if max_x == None or bounds[2] > max_x:
            max_x = bounds[2]
        if max_y == None or bounds[3] > max_y:
            max_y = bounds[3]
    ax.set_xlim(min_x-buff,max_x+buff)
    ax.set_ylim(min_y-buff,max_y+buff)
    ax.set_aspect(1)

def make_axis():
    from pylab import plot, figure
    return figure(1, dpi=90).add_subplot(111), True

def plot_lines(lines, ax=None, color='#6699cc', alpha=1.0):
    import matplotlib.pyplot as pyplot
    show = False
    if ax == None:
        ax, show = make_axis()
    x, y = line.xy
    t = np.linspace(0, 10, len(x))
    lc = LineCollection(lines, cmap=pyplot.get_cmap('hot'),
                               norm=pyplot.Normalize(0, 20))
    ax.add_collection(lc)
    lc.set_array(t)
    lc.set_linewidth(2)
    if show:
        import pylab; pylab.show()

def plot_line(line, ax=None, color='#6699cc', alpha=1.0):
    import matplotlib.pyplot as pyplot
    show = False
    if ax == None:
        ax, show = make_axis()
    x, y = line.xy
    lines = []
    for p in range(0, len(x), 2):
        lines.append(LineString([(x[p], y[p]), (x[p+1], y[p+1])]))
    t = np.linspace(12, 20, len(lines))
    lc = LineCollection(lines, cmap=pyplot.get_cmap('jet'),
                               norm=pyplot.Normalize(0, 20))
    ax.add_collection(lc)
    lc.set_array(t)
    lc.set_linewidth(2)
    if show:
        import pylab; pylab.show()

def plot_coords(coords, ax=None, color='#999999', alpha=1.0):
    show = False
    if ax == None:
        ax, show = make_axis()
    x, y = coords.xy
    ax.plot(x, y, 'o', color=color, alpha=alpha, zorder=1)
    if show:
        import pylab; pylab.show()

def plot_polygon(polygon, ax=None, color='#999999', alpha=1.0):
    show = False
    if ax == None:
        ax, show = make_axis()
    x,y = polygon.exterior.xy
    ax.plot(x,y, color=color, alpha=alpha)
    ax.set_xlim(min(x)-1,max(x)+1)
    ax.set_ylim(min(y)-1,max(y)+1)
    ax.set_aspect(1)
    if show:
        import pylab; pylab.show()


##### Tests #####
if __name__ == '__main__':
    ext = [(1, 1), (-3, 11), (4, 16), (10, 10)]
    polygon = Polygon(ext)
    polygon_points = np.array(polygon.exterior)
    
    rt = rotation_tf_from_longest_edge(polygon)
    print rt.angle
    print rt.w
    
    tf_points = rotate_to(polygon_points, rt)
    tf_polygon = ndarray2polygon(tf_points)
    
    ax, _ = make_axis()
    plot_polygon(polygon, ax, color='blue')
    plot_polygon(tf_polygon, ax, color='red')
    ax.axvline(x=0, color='black')
    ax.axhline(y=0, color='black')
    zoom_extents(ax, [polygon,tf_polygon])
    import pylab; pylab.show()
    
