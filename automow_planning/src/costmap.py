#!/usr/bin/env python

import os
import numpy as np

WHITE = '\033[97m'
CYAN = '\033[96m'
MAGENTA = '\033[95m'
BLUE = '\033[94m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
GRAY = '\033[90m'
ENDC = '\033[0m'

NOTRVRSE = 255
CUTGRASS = 0

COVERAGE_ITERATION_LIMIT = 100

class Costmap2D:
    """Two Dimensional costmap"""
    def __init__(self):
        self.__data = None
        self.exterior_obstacles = []
        self.cutgrass = [0]
        self.leading_edge = []
        self.use_color = True
    
    def __str__(self):
        """docstring for __str__"""
        result = "\n"
        for x in range(self.x_dim):
            for y in range(self.y_dim):
                if self.use_color:
                    if (x,y) in self.exterior_obstacles:
                        result += GRAY
                        result += "%3s"%self.__data[x][y]
                        result += ENDC
                    elif self.__data[x][y] == NOTRVRSE:
                        result += WHITE
                        result += "%3s"%self.__data[x][y]
                        result += ENDC
                    elif self.__data[x][y] == 1:
                        result += GREEN
                        result += "%3s"%self.__data[x][y]
                        result += ENDC
                    elif self.__data[x][y] == 2:
                        result += BLUE
                        result += "%3s"%self.__data[x][y]
                        result += ENDC
                    elif self.__data[x][y] == 3:
                        result += CYAN
                        result += "%3s"%self.__data[x][y]
                        result += ENDC
                    elif self.__data[x][y] == 4:
                        result += YELLOW
                        result += "%3s"%self.__data[x][y]
                        result += ENDC
                    elif self.__data[x][y] == 5:
                        result += MAGENTA
                        result += "%3s"%self.__data[x][y]
                        result += ENDC
                    elif self.__data[x][y] >= 6:
                        result += RED
                        result += "%3s"%self.__data[x][y]
                        result += ENDC
                    else:
                        result += WHITE
                        result += "%3s"%self.__data[x][y]
                        result += ENDC
                else:
                    result += "%3s"%self.__data[x][y]
            result += "\n"
        return result
    
    def useColor(self, value):
        """docstring for useColor"""
        self.use_color = value
    
    def setData(self, data):
        """asdf"""
        self.__data = data
        self.x_dim = len(data)
        self.y_dim = len(data[0])
    
    def getNeighbors(self, x, y):
        """docstring for getNeighbors"""
        d = self.__data
        neighbors = []
        if x > 0:
            neighbors.append((x-1,y,d[x-1][y]))
        if x < self.x_dim-1:
            neighbors.append((x+1,y,d[x+1][y]))
        if y > 0:
            neighbors.append((x,y-1,d[x][y-1]))
        if y < self.y_dim-1:
            neighbors.append((x,y+1,d[x][y+1]))
        return neighbors
    
    def isExteriorObstacle(self, x, y):
        """docstring for isSurrounded"""
        neighbors = self.getNeighbors(x,y)
        for x,y,neighbor in neighbors:
            if neighbor != NOTRVRSE:
                return True
        return False
    
    def stepBrushfire(self):
        """docstring for stepBrushfire"""
        if self.leading_edge == []: # First step
            self.cutgrass = []
            for x in range(self.x_dim):
                for y in range(self.y_dim):
                    current_cell = self.__data[x][y]
                    if current_cell == CUTGRASS:
                        self.cutgrass.append((x,y))
                    if not self.isExteriorObstacle(x,y) or current_cell != NOTRVRSE:
                        continue
                    self.exterior_obstacles.append((x,y))
            for x,y in self.exterior_obstacles:
                current_cell = self.__data[x][y]
                neighbors = self.getNeighbors(x,y)
                for xn,yn,neighbor in neighbors:
                    if neighbor != NOTRVRSE and neighbor == 0:
                        self.__data[xn][yn] = 1
                        self.leading_edge.append((xn,yn))
                        try:
                            self.cutgrass.remove((xn,yn))
                        except ValueError:
                            pass
        else:
            le = list(self.leading_edge)
            self.leading_edge = []
            for x,y in le:
                # self.__data[x][y] += 1
                current_cell = self.__data[x][y]
                neighbors = self.getNeighbors(x,y)
                for xn,yn,neighbor in neighbors:
                    if neighbor == 0:
                        self.__data[xn][yn] = current_cell + 1
                        self.leading_edge.append((xn,yn))
                        try:
                            self.cutgrass.remove((xn,yn))
                        except ValueError:
                            pass
    
    def planCoverage(self):
        """docstring for planCoverage"""
        count = 0
        while len(self.cutgrass) > 0 and count != COVERAGE_ITERATION_LIMIT:
            self.stepBrushfire()
            count += 1
        if count == COVERAGE_ITERATION_LIMIT:
            return False
        return True
    


