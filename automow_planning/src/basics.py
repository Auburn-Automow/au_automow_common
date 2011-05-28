#!/opt/local/bin/python
import os
import PIL
import StringIO
import math
import sys
import exceptions
import roslib
roslib.load_manifest('automow_planning')
import nav_msgs.msg
import numpy as np
import multiprocessing as mp

OC = nav_msgs.msg.OccupancyGrid

# Width in meters
map_width = 20.0
# Height in meters
map_height = 20.0
# Resolution in meters per cell
resolution = 0.05

#******* Helper Functions *******
# Converts meteres to a number of cells that respresent that distance
def meters_to_cells(meters):
    return int(resolution / meters)

NOT_ACCESSABLE = -1
CUT = -2
UNCUT = -3
DONT_CUT = -4

def calc_distance(x, y, u, v):
    "Returns the euclidean(sp?) distance between two points as an int"
    return int(math.floor(math.sqrt((x-u)*(x-u) + (y-v)*(y-v))))

def minimizer(args):
    "Recursive Minimizer takes a list of 2 or more numpy arrays and minimizes them."
    r = args[0]
    i = 1
    for i in range(len(args)):
        r = np.minimum(r, args[i])
    return r


def _calc_distances(pl_x_y_i_j):
    "Calculateds all of the distances for the map"
    # TODO: Needs to shortcut if its calculating stuff it doesnt need
    point_list, max_x, max_y, i, j = pl_x_y_i_j
    results = np.array([[x*y for y in range(max_y)] for x in range(max_x)])
    for x in range(max_x):
        for y in range(max_y):
            if x == i and y == j:
                continue
            if point_list[x][y] == NOT_ACCESSABLE:
                results[x][y] = -1
                continue
            results[x][y] = calc_distance(i, j, x, y)
    return results

class Map(object):
    "A map of the field"
    def __init__(self, x, y, default_value=None):
        "params x, y are the sizes and the default value is the occupancy default value"
        self.x = x
        self.y = y
        self._pool = mp.Pool(processes=2)
        self._default_value = default_value
        self._point_list = np.array([[default_value for y in range(self.y)] for x in range(self.x)])
        self._cost_map = np.array([[0.0 for y in range(self.y)] for x in range(self.x)])
    
    def set_point(self, x, y, value):
        ""
        self._point_list[x][y] = value
        self._cost_map[x][y] = value
    
    def get_point(self, x, y):
        "Gets a point and its cost"
        try:
            return (self.get_occupancy(x, y), self._cost_map[x][y])
        except KeyError as e: 
            return (self._default_value, -1)
    
    def get_occupancy(self, x, y):
        "Returns the occupancy state of a node in the map"
        if x > self.x or y > self.y:
            raise exceptions.IndexError, 'Index ' + str(x) + ',' + str(y) + ' out of bounds.'
        try:
            return self._point_list[x][y]
        except KeyError as e: 
            return self._default_value
    
    def print_occupancy(self):
        print "\n    Occupancy Grid"
        val = self._point_list.T
        for i in range(val.shape[0]-1, -1, -1):
            print '{0:3d}:'.format(i),
            for j in range(val.shape[1]):
                if val[i][j] == NOT_ACCESSABLE:
                    print '   .',
                elif val[i][j] == CUT:
                    print '   @',
                elif val[i][j] == UNCUT:
                    print '   #',
                else:
                    print '{0:4d}'.format(val[i][j]),
            print
        print '   ',
        for j in range(val.shape[1]):
            print '   _',
        print
        print '   ',
        for j in range(val.shape[1]):
            print '{0:4d}'.format(j),
        print
    
    def print_costmap(self):
        val = self._cost_map.T
        print "\n    Costmap"
        for i in range(val.shape[0]-1, -1, -1):
            print '{0:3d}:'.format(i),
            for j in range(val.shape[1]):
                if val[i][j] == UNCUT:
                    print '   .',
                elif val[i][j] == NOT_ACCESSABLE:
                    print '   @',
                else:
                    print "{0:4d}".format(int(val[i][j])),
            print
        print '    ',
        for j in range(val.shape[1]):
            print '   _',
        print '\n    ',
        for j in range(val.shape[1]):
            print '{0:4d}'.format(j),
        print "\n\n"
    
    def visualize(self, filename=None):
        if filename:
            f = open(filename, "w");
        else:
            f = StringIO.StringIO()

        if f.closed:
            print "Error"
            return

        f.write("P6\n")
        f.write("{0:d} {1:d} 255\n".format(self.x, self.y))

        for y in range(self.y-1, -1, -1):
            for x in range(self.x):
                c = 0;
                if self.isVoronoi(x,y):
                    f.write(chr(255))
                    f.write(chr(0))
                    f.write(chr(0))
                elif self.data[x][y].sqdist == 0:
                    f.write(chr(0))
                    f.write(chr(0))
                    f.write(chr(0))
                else:
                    f = 80 + (self.data[x][y].dist * 5)
                    if f > 255:
                        f=255
                    if f < 0:
                        f=0
                    f.write(chr(f))
                    f.write(chr(f))
                    f.write(chr(f))
        if filename:
            f.close()
        else:
            return f.getvalue()
        
    def update_costmap(self, x1, y1, x2, y2):
        """
        Generates a costmap for a given dataset
        """
        self._cost_map = np.array([[self.x*self.y for y in range(self.y)] for x in range(self.x)])
        r = self._pool.map(_calc_distances, [(self._point_list, self.x, self.y, x, y) for y in range(self.y) for x in range(self.x) if self._point_list[x][y] == NOT_ACCESSABLE])
        self._cost_map = minimizer(r)

def main():
    print 'starting'
    x = 30
    y = 30
    og = Map(x, y, default_value = UNCUT)
    
    # make the border
    for w in range(x):
        og.set_point(w, 0, NOT_ACCESSABLE)
        og.set_point(w, y-1, NOT_ACCESSABLE)
    for v in range(y):
        og.set_point(0, v, NOT_ACCESSABLE)
        og.set_point(x-1, v, NOT_ACCESSABLE)
    # og.set_point(5, 5, NOT_ACCESSABLE)
    # og.set_point(5, 6, NOT_ACCESSABLE)
    # og.set_point(6, 5, NOT_ACCESSABLE)
    # og.set_point(6, 6, NOT_ACCESSABLE)
    # og.set_point(15,15, NOT_ACCESSABLE)
    # og.set_point(18, 18, NOT_ACCESSABLE)
    # og.set_point(18,17, NOT_ACCESSABLE)
    # og.set_point(17,18, NOT_ACCESSABLE)
    # og.set_point(16,18, NOT_ACCESSABLE)
    # og.set_point(17,17, NOT_ACCESSABLE)
    # og.set_point(18,16, NOT_ACCESSABLE)
    og.set_point(4,  3, NOT_ACCESSABLE)
    og.set_point(5,  3, NOT_ACCESSABLE)
    og.set_point(6,  3, NOT_ACCESSABLE)
    og.set_point(11, 7, NOT_ACCESSABLE)
    og.set_point(12, 7, NOT_ACCESSABLE)
    og.set_point(13, 7, NOT_ACCESSABLE)
    
    og.update_costmap(0, 0, og.x-1, og.y-1)
    
    # print og.get_point(2, 4), og.get_point(0, 1) 
    og.print_occupancy()
    print
    og.print_costmap()
    # print "print along the x:"
    # print repr(og.generate_sub_regions(range_max=x, length_max=y, axis=('x','y')))
    # print "\n", "printing along the y:"
    # print repr(og.generate_sub_regions(range_max=y, length_max=x, axis=('y','x')))
    # print 'test complete'
    

if __name__ == '__main__':
    main()

