import numpy as np
from matplotlib.lines import Line2D


def rad2deg(angle):
    return 180 / np.pi * angle


def deg2rad(angle):
    return np.pi / 180.0 * angle


def wrapToPi(angle):
    return np.mod(angle + np.pi,  2.0 * np.pi) - np.pi


class Controller(object):
    def __init__(self, max_v, max_w):
        self.max_v = max_v
        self.max_w = max_w
        self._lateral_error_gain = 1.0
        self._heading_error_gain = 1.0
        self.translation = np.array([0.0, 0.0])
        self.rotation = np.eye(2)

    @property
    def lateral_error_gain(self):
        """ The lateral_error_gain property."""
        return self._lateral_error_gain

    @lateral_error_gain.setter
    def lateral_error_gain(self, value):
        self._lateral_error_gain = value

    @property
    def heading_error_gain(self):
        """The heading_error_gain property."""
        return self._heading_error_gain

    @heading_error_gain.setter
    def heading_error_gain(self, value):
        self._heading_error_gain = value

    def perpindicular_distance(self, point):
        (m, b) = (self.path.slope, self.path.intercept)
        b = -m * self.path.start[0] + self.path.start[1]
        distance = np.abs(m * point[0] - point[1] + b) / np.sqrt(m ** 2 + 1)
        if point[1] < m * point[0] + b:
            distance *= -1
        return distance

    def heading_error(self, heading):
        return wrapToPi(heading - self.path.heading)

    def calculate_effort(self, position, v=None):
        if v is None:
            v = self.max_v

        #print (self.perpindicular_distance(position[:2]), self.heading_error(position[2]))
        if self.path.heading > np.pi:
            w = (- self._lateral_error_gain * -self.perpindicular_distance(position[:2])
                 - self._heading_error_gain * self.heading_error(position[2]))
        else:
            w = (- self._lateral_error_gain * self.perpindicular_distance(position[:2])
                 - self._heading_error_gain * self.heading_error(position[2]))

        if v > self.max_v:
            v = self.max_v
        if v < -self.max_v:
            v = -self.max_v

        if w > self.max_w:
            w = self.max_w
        if w < -self.max_w:
            w = -self.max_w

        return (v, w)

    def set_path(self, line):
        self.path = line


class RobotModel(object):
    def __init__(self, wheel_radius, wheel_base, x_0=0, y_0=0, th_0=0, dt=0.1):
        self.pose = np.array([x_0, y_0, th_0], dtype=np.float64)
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.dt = dt

    def move(self, v, w):
        x_dot = self.dt * v * np.cos(self.pose[2] + self.dt * w / 2.0)
        y_dot = self.dt * v * np.sin(self.pose[2] + self.dt * w / 2.0)
        th_dot = self.dt * w
        self.pose += np.array([x_dot, y_dot, th_dot])
        return self.pose

    @property
    def position(self):
        """ The position property."""
        return self.pose[:2]

    @property
    def quiver(self):
        return (self.pose[0], self.pose[1], np.cos(self.pose[2]), np.sin(self.pose[2]))


class LineModel(object):
    """
    Represents the self.path.to track to
    """
    def __init__(self, start, end):
        self.start = start
        self.end = end

    @property
    def length(self):
        return np.sqrt((self.start[0] - self.end[0]) ** 2 +
                       (self.start[1] - self.end[1]) ** 2)

    @property
    def heading(self):
        return np.arctan2(self.end[1] - self.start[1], self.end[0] - self.start[0])

    @property
    def slope(self):
        return (self.end[1] - self.start[1]) / float(self.end[0] - self.start[0])

    @property
    def intercept(self):
        return -self.slope * self.start[0] + self.start[1]

    def asLine2D(self):
        return Line2D([self.start[0], self.end[0]], [self.start[1], self.end[1]])
