#!/usr/bin/env python

import os
import numpy as np
from math import *

WHITE = '\033[97m'
CYAN = '\033[96m'
MAGENTA = '\033[95m'
BLUE = '\033[94m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
GRAY = '\033[90m'
GRAY_BG = '\033[100m'
WHITE_BG = '\033[107m'
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
        self.consumed_cells = []
        self.use_color = True
        self.robot_position = (-1,-1)
        self.target_position = None
        self.sorted_consumables = {}
        self.consumption_complete = False
    
    def __str__(self):
        """docstring for __str__"""
        result = "\n"
        for x in range(self.x_dim):
            for y in range(self.y_dim):
                if self.use_color:
                    color = None
                    if (x,y) in self.exterior_obstacles:
                        color = GRAY
                    elif self.__data[x][y] == NOTRVRSE:
                        color = WHITE
                    elif self.__data[x][y] == 1:
                        color = GREEN
                    elif self.__data[x][y] == 2:
                        color = BLUE
                    elif self.__data[x][y] == 3:
                        color = CYAN
                    elif self.__data[x][y] == 4:
                        color = YELLOW
                    elif self.__data[x][y] == 5:
                        color = MAGENTA
                    elif self.__data[x][y] >= 6:
                        color = RED
                    else:
                        color = WHITE
                    if (x,y) in self.consumed_cells:
                        color += GRAY_BG
                    if (x,y) == self.robot_position:
                        color += WHITE_BG
                    result += color
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
    
    def getData(self):
        """docstring for getData"""
        return self.__data
    
    def getCardinalNeighbors(self, x, y):
        """docstring for getNeighbors"""
        d = self.__data
        neighbors = []
        if x > 0:
            neighbors.append(self._getNeighbor(x-1,y))
        if x < self.x_dim-1:
            neighbors.append(self._getNeighbor(x+1,y))
        if y > 0:
            neighbors.append(self._getNeighbor(x,y-1))
        if y < self.y_dim-1:
            neighbors.append(self._getNeighbor(x,y+1))
        return neighbors
    
    def getNeighbors(self, x, y):
        """docstring for getNeighbors"""
        d = self.__data
        neighbors = []
        if x > 0 and y > 0:
            neighbors.append(self._getNeighbor(x-1,y-1))
        if x < self.x_dim-1 and y > 0:
            neighbors.append(self._getNeighbor(x+1,y-1))
        if x < self.x_dim-1 and y < self.y_dim-1:
            neighbors.append(self._getNeighbor(x+1,y+1))
        if x > 0 and y < self.y_dim-1:
            neighbors.append(self._getNeighbor(x-1,y+1))
        return neighbors+self.getCardinalNeighbors(x,y)
    
    def _getNeighbor(self,x,y):
        """docstring for _getNeighbor"""
        return x,y,self.__data[x][y]
    
    def isExteriorObstacle(self, x, y):
        """docstring for isSurrounded"""
        neighbors = self.getCardinalNeighbors(x,y)
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
                neighbors = self.getCardinalNeighbors(x,y)
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
                neighbors = self.getCardinalNeighbors(x,y)
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
    
    def setRobotPosition(self, x, y):
        """docstring for setRobotPosition"""
        self.robot_position = (int(floor(x)), int(floor(y)))
        self.consumed_cells.append(self.robot_position)
        if self.target_position != None and self.target_position == self.robot_position:
            self.target_position = None
    
    def getNextPosition(self):
        """docstring for getNextPosition"""
        # return self._getNextPositionWander()
        return self._getNextPositionBurnDown()
    
    def _getNextPositionBurnDown(self):
        """docstring for _getNextPositionBurnDown"""
        if self.consumption_complete:
            self.consumption_complete = False
            return None
        self._generateSortedConsumables()
        if self.target_position != None:
            self.consumed_cells(self.target_position)
            self.target_position = None
        if self.target_position == None:
            target_num = max(self.sorted_consumables.keys())
            target_positions = self.sorted_consumables[target_num]
            target_positions = sorted(target_positions, key=lambda pos: abs(sqrt( (pos[0]-self.robot_position[0])**2 + (pos[1]-self.robot_position[1])**2 )))
            (x,y,value) = target_positions[0]
            self.target_position = (x,y)
            if len(target_positions) == 1:
                del self.sorted_consumables[target_num]
            else:
                self.sorted_consumables[target_num].remove(target_positions[0])
            if self.sorted_consumables == {}:
                self.consumption_complete = True
        return self.target_position
    
    def _generateSortedConsumables(self):
        """docstring for _generateSortedConsumables"""
        if self.sorted_consumables == {}:
            for x in range(self.x_dim):
                for y in range(self.y_dim):
                    value = int(self.__data[x][y])
                    if value != NOTRVRSE:
                        if value not in self.sorted_consumables:
                            self.sorted_consumables[value] = []
                        self.sorted_consumables[value].append((x,y,value))
    
    def _getNextPositionWander(self):
        """docstring for _getNextPositionWander"""
        neighbors = self.getNeighbors(*self.robot_position)
        possible_next_positions = []
        for xn,yn,neighbor in neighbors:
            if (xn, yn) in self.consumed_cells or neighbor == NOTRVRSE:
                continue
            possible_next_positions.append((xn,yn,neighbor))
        if len(possible_next_positions) == 0: # All neighbors are consumed
            for xn2,yn2,neighbor2 in neighbors:
                if neighbor2 != NOTRVRSE:
                    possible_next_positions.append((xn2,yn2,neighbor2))
            if len(possible_next_positions) == 0: # No traversable paths!
                print "ERROR: No traversable paths!"
                return None
        xs,ys,values = zip(*possible_next_positions)
        index = values.index(max(values))
        return xs[index], ys[index]
    


