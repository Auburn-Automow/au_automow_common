from matplotlib import pyplot
from shapely.geometry import Point, Polygon, LineString, MultiLineString, MultiPoint
from descartes.patch import PolygonPatch
import math
import numpy as np
import copy

from pprint import pprint

def plot_coords(ax, ob, color='#999999'):
    x, y = ob.xy
    ax.plot(x, y, 'o', color=color, zorder=1)

def plot_line(ax, ob):
    x, y = ob.xy
    ax.plot(x, y, color='#6699cc', alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)

def plot_lines(ax, ob):
    for line in ob:
        x, y = line.xy
        ax.plot(x, y, color='#121212', alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)

def generate_intersections(poly, width):
    "Subdivide a filed into coverage lines."
    starting_breakdown = poly.bounds[0:2]
    line = LineString([starting_breakdown, (starting_breakdown[0],
                                            starting_breakdown[1] +
                                            poly.bounds[3] - poly.bounds[1])])
    bounded_line = polygon.intersection(line)
    lines = [bounded_line]

    x = 0
    while 1:
        x += 1
        bounded_line = line.parallel_offset(x * width, 'right')
        if polygon.intersects(bounded_line):
            bounded_line = polygon.intersection(bounded_line)
            lines.append(bounded_line)
        else:
            break
    return lines

def sort_to(point, list):
    "Sorts a set of points by distance to a point"
    l = copy.deepcopy(list)
    l.sort(lambda x, y: cmp(x.distance(Point(*point)),
                            y.distance(Point(*point))))
    return l

def get_furthest(ps, origin):
    "Get a point along a line furthest away from a given point"
    orig_point = Point(*origin)
    return sorted(ps, lambda x, y: cmp(orig_point.distance(Point(*x)),
                                       orig_point.distance(Point(*y))))

def order_points(lines, initial_origin):
    "Return a list of points in a given coverage path order"
    origin = initial_origin
    results = []
    while origin != None:
        if len(lines):
            lines = sort_to(origin, lines)
            f = lines[0]
            lines = lines[1:]
            if type(f) == MultiLineString:
                for ln in f:
                    lines.append(ln)
                continue
            if type(f) == Point or type(f) == MultiPoint:
                continue
            xs, ys = f.xy
            ps = zip(xs, ys)
            (start, origin_) = get_furthest(ps, origin)
            results.append((origin, start))
            results.append((start, origin_))
            origin = origin_
        else:
            origin = None
    return results

def decompose(polygon, origin=None, width=1.0):
    """
    Decompose the field into a list of points to cover the field.
    """
    p = generate_intersections(polygon, width)
    if origin == None:
        return order_points(p, polygon.bounds[0:2])
    else:
        return order_points(p, origin)

if __name__ == '__main__':
    fig = pyplot.figure(1, dpi=90)
    # 3: invalid polygon, ring touch along a line
    ax = fig.add_subplot(111)

    ext = [(0, 0), (2, 5), (0, 11), (10, 11), (15, 5), (10, 0), (0, 0)]
    inter = [(3, 3), (3, 7), (7, 7), (7,3), (3, 3)]
    polygon = Polygon(ext, [inter])
    plot_coords(ax, polygon.exterior)

    patch = PolygonPatch(polygon, facecolor='#6699cc',
                         edgecolor='#235612', alpha=0.5, zorder=2)
    ax.add_patch(patch)

    bounds = polygon.bounds
    xrange = (bounds[0]-1, bounds[2]+1)
    yrange = (bounds[1]-1, bounds[3]+1)
    ax.set_xlim(*xrange)
    ax.set_ylim(*yrange)
    ax.set_aspect(1)

    r = decompose(polygon)

    pprint(r)

    ll = MultiLineString(r)
    plot_lines(ax, ll)

    pyplot.show()
