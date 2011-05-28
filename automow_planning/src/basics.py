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
            if point_list[x][y] != UNCUT:
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
        self._pool = mp.Pool(processes=8)
        self._default_value = default_value
        self._point_list = np.array([[default_value for y in range(self.y)] for x in range(self.x)])
        self._cost_map = np.array([[0.0 for y in range(self.y)] for x in range(self.x)])
    
    def set_point(self, x, y, value):
        ""
        self._point_list[x][y] = value
        self._cost_map[x][y] = value
    
    def set_points(self, x_range, y_range, value):
        ""
        for x in x_range:
            for y in y_range:
                self.set_point(x, y, value)

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
            print '{0:3d}|'.format(i),
            for j in range(val.shape[1]):
                if val[i][j] == NOT_ACCESSABLE:
                    print '   .',
                elif val[i][j] == CUT:
                    print '   @',
                elif val[i][j] == UNCUT:
                    print '   #',
                elif val[i][j] == DONT_CUT:
                    print '   $',
                else:
                    print '{0:4d}'.format(val[i][j]),
            print
        print '   ',
        for j in range(val.shape[1]):
            print ' ___',
        print
        print '   ',
        for j in range(val.shape[1]):
            print '{0:4d}'.format(j),
        print
    
    def print_costmap(self):
        val = self._cost_map.T
        print "\n    Costmap"
        for i in range(val.shape[0]-1, -1, -1):
            print '{0:3d}|'.format(i),
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
            print ' ___',
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
                        f = 255
                    if f < 0:
                        f = 0
                    f.write(chr(f))
                    f.write(chr(f))
                    f.write(chr(f))
        if filename:
            f.close()
        else:
            return f.getvalue()
        
    def update_costmap(self):
        """
        Generates a costmap for a given dataset

        1 2 3
        4 5 6
        7 8 9

1 = -1, +1
2 = 0, +1
3 = +1, +1
4 = -1, 0
5 = NA
6 = +1, 0
7 = -1, -1
8 = 0, -1
9 = +1, -1

        """
        def check_point(x,y):
            try:
                # if self._point_list[x][y] != UNCUT and UNCUT in self._point_list[x-1:y-1,x+1:y+1]:
                if self._point_list[x][y] != UNCUT and (\
                       self._point_list[x-1][y+1] == UNCUT or \
                       self._point_list[x][y+1] == UNCUT or \
                       self._point_list[x+1][y+1] == UNCUT or \
                       self._point_list[x-1][y] == UNCUT or \
                       self._point_list[x+1][y] == UNCUT or \
                       self._point_list[x-1][y-1] == UNCUT or \
                       self._point_list[x][y-1] == UNCUT or \
                       self._point_list[x+1][y-1] == UNCUT):
                    return True
            except IndexError:
                pass
            return False

        self._cost_map = np.array([[self.x*self.y for y in range(self.y)] for x in range(self.x)])
        r = self._pool.map(_calc_distances, [(self._point_list, self.x, self.y, x, y) for y in range(self.y) for x in range(self.x) if check_point(x,y)])
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
    # og.set_point(4,  3, NOT_ACCESSABLE)
    # og.set_point(5,  3, NOT_ACCESSABLE)
    # og.set_point(6,  3, NOT_ACCESSABLE)
    # og.set_point(11, 8, NOT_ACCESSABLE)
    # og.set_point(12, 8, NOT_ACCESSABLE)
    # og.set_point(13, 8, NOT_ACCESSABLE)
    # og.set_point(11, 7, NOT_ACCESSABLE)
    # og.set_point(12, 7, NOT_ACCESSABLE)
    # og.set_point(13, 7, NOT_ACCESSABLE)

    # Draw the field
    og.set_point( 3,  8, DONT_CUT)
    og.set_point( 3,  9, DONT_CUT)
    og.set_point( 3, 10, DONT_CUT)
    og.set_point( 3, 11, DONT_CUT)
    og.set_point( 3, 12, DONT_CUT)
    og.set_point( 3, 13, DONT_CUT)
    og.set_point( 3, 14, DONT_CUT)
    og.set_point( 3, 15, DONT_CUT)
    og.set_point( 3, 16, DONT_CUT)
    og.set_point( 3, 17, DONT_CUT)
    og.set_point( 3, 18, DONT_CUT)
    og.set_point( 3, 19, DONT_CUT)
    og.set_point( 3, 20, DONT_CUT)
    og.set_point( 3, 21, DONT_CUT)
    og.set_point( 3, 22, DONT_CUT)
    og.set_point( 3, 23, DONT_CUT)
    og.set_point( 4, 23, DONT_CUT)
    og.set_point( 5, 23, DONT_CUT)
    og.set_point( 6, 23, DONT_CUT)
    og.set_point( 7, 23, DONT_CUT)
    og.set_point( 8, 23, DONT_CUT)
    og.set_point( 9, 23, DONT_CUT)
    og.set_point(10, 23, DONT_CUT)
    og.set_point(11, 23, DONT_CUT)
    og.set_point(12, 23, DONT_CUT)
    og.set_point(13, 23, DONT_CUT)
    og.set_point(14, 23, DONT_CUT)
    og.set_point(15, 23, DONT_CUT)
    og.set_point(16, 23, DONT_CUT)
    og.set_point(17, 23, DONT_CUT)
    og.set_point(18, 23, DONT_CUT)
    og.set_point(19, 23, DONT_CUT)
    og.set_point(20, 23, DONT_CUT)
    og.set_point(21, 23, DONT_CUT)
    og.set_point(22, 23, DONT_CUT)
    og.set_point(23, 23, DONT_CUT)
    og.set_point(24, 23, DONT_CUT)
    og.set_point(25, 23, DONT_CUT)
    og.set_point(25,  8, DONT_CUT)
    og.set_point(25,  9, DONT_CUT)
    og.set_point(25, 10, DONT_CUT)
    og.set_point(25, 11, DONT_CUT)
    og.set_point(25, 12, DONT_CUT)
    og.set_point(25, 13, DONT_CUT)
    og.set_point(25, 14, DONT_CUT)
    og.set_point(25, 15, DONT_CUT)
    og.set_point(25, 16, DONT_CUT)
    og.set_point(25, 17, DONT_CUT)
    og.set_point(25, 18, DONT_CUT)
    og.set_point(25, 19, DONT_CUT)
    og.set_point(25, 20, DONT_CUT)
    og.set_point(25, 21, DONT_CUT)
    og.set_point(25, 22, DONT_CUT)
    og.set_point( 4,  8, DONT_CUT)
    og.set_point( 5,  8, DONT_CUT)
    og.set_point( 6,  8, DONT_CUT)
    og.set_point( 7,  8, DONT_CUT)
    og.set_point( 8,  8, DONT_CUT)
    og.set_point( 9,  8, DONT_CUT)
    og.set_point(10,  8, DONT_CUT)
    og.set_point(11,  8, DONT_CUT)
    og.set_point(12,  8, DONT_CUT)
    og.set_point(13,  8, DONT_CUT)
    og.set_point(14,  8, DONT_CUT)
    og.set_point(15,  8, DONT_CUT)
    og.set_point(16,  8, DONT_CUT)
    og.set_point(17,  8, DONT_CUT)
    og.set_point(18,  8, DONT_CUT)
    og.set_point(19,  8, DONT_CUT)
    og.set_point(20,  8, DONT_CUT)
    og.set_point(21,  8, DONT_CUT)
    og.set_point(22,  8, DONT_CUT)
    og.set_point(23,  8, DONT_CUT)
    og.set_point(24,  8, DONT_CUT)


    # Draw the flowerbed-ish
    og.set_point(9,  13, NOT_ACCESSABLE)
    og.set_point(9,  14, NOT_ACCESSABLE)
    og.set_point(9,  15, NOT_ACCESSABLE)
    og.set_point(9,  16, NOT_ACCESSABLE)
    og.set_point(9,  17, NOT_ACCESSABLE)
    og.set_point(10,  13, NOT_ACCESSABLE)
    og.set_point(10,  14, NOT_ACCESSABLE)
    og.set_point(10,  15, NOT_ACCESSABLE)
    og.set_point(10,  16, NOT_ACCESSABLE)
    og.set_point(10,  17, NOT_ACCESSABLE)
    og.set_point(11,  13, NOT_ACCESSABLE)
    og.set_point(11,  14, NOT_ACCESSABLE)
    og.set_point(11,  15, NOT_ACCESSABLE)
    og.set_point(11,  16, NOT_ACCESSABLE)
    og.set_point(11,  17, NOT_ACCESSABLE)


    og.set_points(range(1,3), range(1,29), DONT_CUT)
    og.set_points(range(3,26), range(1,8), DONT_CUT)
    og.set_points(range(3,26), range(24,29), DONT_CUT)
    og.set_points(range(26,29), range(1,29), DONT_CUT)
    
    og.update_costmap()
    
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

