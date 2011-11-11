#!/usr/bin/env python

"""
Contains utilities for working with maps and field shapes.
"""

import numpy as np
from PIL import Image
from shapely.geometry import Polygon
from shapely.ops import polygonize
from math import cos, sin

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

class RotationTransform:
    """Represents a rotational transform"""
    def __init__(self, angle):
        self.angle = angle
        self.irm = np.mat([[cos(self.angle), -sin(self.angle), 0],
                           [sin(self.angle), cos(self.angle),  0],
                           [0,               0,                1]])
    

def rotate_to(points, rotation_transform):
    """Rotates an ndarray of given points(x,y) to a given rotation"""
    if type(points) != np.ndarray:
        raise TypeError("rotate_to: takes an numpy.ndarray")
    new_points = []
    for point in points:
        point_mat = np.mat([[point[0]],[point[1]],[0]])
        new_point = rt.irm * point_mat
        new_points.append(np.array(new_point[:-1].T))
    return np.squeeze(np.array(new_points))

def rotate_from(points, rotation_transform):
    """Rotate an ndarray of given points(x,y) from a given rotation"""
    if type(points) != np.ndarray:
        raise TypeError("rotate_from: takes an numpy.ndarray")
    new_points = []
    for point in points:
        point_mat = np.mat([[point[0]],[point[1]],[0]])
        new_point = rt.irm.I * point_mat
        new_points.append(np.array(new_point[:-1].T))
    return np.squeeze(np.array(new_points))

def plot_polygon(polygon, show = True):
    import pylab
    from pylab import plot, figure
    ax = figure(1, dpi=90).add_subplot(111)
    x,y = polygon.exterior.xy
    ax.plot(x,y)
    ax.set_xlim(min(x)-1,max(x)+1)
    ax.set_ylim(min(y)-1,max(y)+1)
    ax.set_aspect(1)
    if show:
        pylab.show()

def ndarray2polygon(points):
    if type(points) != np.ndarray:
        raise TypeError("ndarray2polygon: takes an numpy.ndarray")
    new_tuples = []
    for point in points:
        new_tuples.append((point[0],point[1]))
    return Polygon(new_tuples)

if __name__ == '__main__':
    ext = [(0, 0), (-3, 11), (10, 16), (10, 12)]
    polygon = Polygon(ext)
    polygon_points = np.array(polygon.exterior)

    rt = RotationTransform(45)

    tf_points = rotate_to(polygon_points, rt)
    tf_polygon = ndarray2polygon(tf_points)

    un_tf_points = rotate_from(tf_points, rt)
    un_tf_polygon = ndarray2polygon(un_tf_points)

    plot_polygon(polygon)
    plot_polygon(tf_polygon)
    plot_polygon(un_tf_polygon)

