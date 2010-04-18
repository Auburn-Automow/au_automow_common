#!/usr/bin/env python
# encoding: utf-8

"""
Drivers for ArduIMU

Created by John Harrison on 2010-04-16.
"""
__author__ = "John Harrison, William Woodall"
__copyright__ = "Copyright (c) John harrison, William Woodall"

import roslib; roslib.load_manifest('arduimu')
import rospy
from sensor_msgs.msg import Imu

from serial import Serial
import re

class ImuDriver(object):
  __slots__ = ['ser', 'pub', 'mode', 'rdy_marker', 'msg_scanner']
  def __init__(self, tty_file = "/dev/ttyUSB0"):
    self.ser = Serial(tty_file, 115200, timeout=1)
    self.pub = rospy.Publisher('chatter', Imu)
    rospy.init_node('imu_driver')
    # reference line:
    # p       31384   484     475     510     +282.46 541     +6.9921 520     +359.92 510     P
    self.msg_scanner = re.compile('''
        p \s+
        (?P<number>\d+)
        \s+
        (?P<x>[+\-]?\d+(?:\.\d+)?) # x acceleration
        \s+
        (?P<y>[+\-]?\d+(?:\.\d+)?) # y acceleration
        \s+
        (?P<z>[+\-]?\d+(?:\.\d+)?) # z acceleration
        \s+
        (?P<roll>[+\-]?\d+(?:\.\d+)?)  # roll
        \s+
        (?P<pitch>[+\-]?\d+(?:\.\d+)?)  # pitch 
        \s+
        (?P<yaw>[+\-]?\d+(?:\.\d+)?)  # yaw
        \s+
        P''', re.X)
    self.ser.write('c')
    self.rdy_marker = re.compile('R,\d+\s+P,\d+\s+Y,\d+')
    while True:
      if self.rdy_marker.match(self.ser.readline()):
        break
    self.mode = None
    rospy.loginfo("IMU Ready")

  def setMode(self, mode_id):
    result = None
    if mode_id not in ['p', 'c', 'g']:
      raise Exception("Bad mode_id")

    if self.mode == 'p':
      self.ser.write('p')
      self.mode = None
    if mode_id == 'p':
      rospy.loginfo("Polling...")
      self.ser.write('p')
      self.mode = 'p'
    elif mode_id == 'c':
      rospy.loginfo("Reconfiguring...")
      self.ser.write('c')
      while True:
        if self.rdy_marker.match(self.ser.readline()):
          break
      rospy.loginfo("Done Reconfiguring")
    elif mode_id == 'g':
      rospy.loginfo("Grabbing a single line")
      self.ser.write('g')
      result = self.ser.readline()
    return result

  def broadcaster(self):
    self.setMode('p')
    while not rospy.is_shutdown():
      raw_msg = self.getMsg()
      msg = self.msg_scanner.match(raw_msg)
      if msg:
        print "got: ", msg
        rospy.loginfo('pitch' + msg.group('pitch'))
        self.pub.publish(Imu(angular_velocity = [msg.group('roll'), msg.group('pitch'), msg.group('yaw')], acceleration=[msg.group('x'), msg.group('y'), msg.group('z')]))
        rospy.sleep(1.0/2.0)
      else:
        raise Exception("Bad message from the IMU: " + raw_msg)

  def getMsg(self):
    self.ser.flushOutput()
    if self.mode == 'p':
      return self.ser.readline()
    else:
      self.ser.write('g')
      return self.ser.readline()

if __name__ == '__main__':
  try:
    arduimu = ImuDriver()
    arduimu.broadcaster()
  except rospy.ROSInterruptException: pass
