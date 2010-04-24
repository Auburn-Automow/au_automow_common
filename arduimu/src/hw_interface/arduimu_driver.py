#!/usr/bin/env python
# encoding: utf-8

"""
Drivers for ArduIMU

Created by John Harrison on 2010-04-16.
"""
__author__ = "John Harrison, William Woodall"
__copyright__ = "Copyright (c) John harrison, William Woodall"

from serial import Serial
import re

class ImuBadMessage(Exception):
  def __init__(self, msg):
    self.msg = "Bad Message:\n" + str(msg)

class ImuDriver(object):
  __slots__ = ['ser', 'mode', 'logger', 'rdy_marker', 'msg_scanner']
  def __init__(self, tty_file = "/dev/ttyUSB0", logger = None, timeout = 0.3):
    self.ser = Serial(tty_file, 115200, timeout=timeout)
    self.logger = logger
    # reference line:
    # p       31384   484     475     510     +282.46 541     +6.9921 520     +359.92 510     P
    self.msg_scanner = re.compile(r'''
        p
        \s+
        (?P<number>\d+)
        \s+
        (?P<x>    [+\-]?\d+(?:\.\d+)?) # x acceleration
        \s+
        (?P<y>    [+\-]?\d+(?:\.\d+)?) # y acceleration
        \s+
        (?P<z>    [+\-]?\d+(?:\.\d+)?) # z acceleration
        \s+
        (?P<roll> [+\-]?\d+(?:\.\d+)?) # roll
        \s+
        (?P<pitch>[+\-]?\d+(?:\.\d+)?) # pitch 
        \s+
        (?P<yaw>  [+\-]?\d+(?:\.\d+)?) # yaw
        \s+
        P''', re.X | re.I | re.M)
    self.ser.write('r')
    self.rdy_marker = re.compile('(?:IMU\s+Initialized!)|(?:R,\d+.*P,\d+.*Y,\d+.*,.*X,\d+.*,.*Y,\d+.*,\sZ,\d+.*)')
    self.log("IMU Initializing")
    count = 0
    while True:
      raw_msg = self.ser.readline()
      if self.rdy_marker.match(raw_msg):
        count += 1
      if count == 2:
        break
    self.mode = None
    self.log("IMU Ready")

  def log(self, msg):
    if self.logger:
      self.logger.loginfo(msg)

  def setMode(self, mode_id):
    result = None
    if mode_id not in ['p', 'c', 'g']:
      raise Exception("Bad mode_id")

    if self.mode == 'p':
      self.ser.write('p')
      self.mode = None
    if mode_id == 'p':
      self.log("Polling...")
      self.ser.write('p')
      self.mode = 'p'
    elif mode_id == 'c':
      self.log("Reconfiguring...")
      self.ser.write('c')
      while True:
        if self.rdy_marker.match(self.ser.readline()):
          break
      self.log("Done Reconfiguring")
    elif mode_id == 'g':
      self.log("Grabbing a single line")
      self.ser.write('g')
      result = self.ser.readline()
    return result

  def getMsg(self):
    raw_msg = self.getMsgRaw()
    msg = self.msg_scanner.match(raw_msg.strip())
    if msg:
      return msg
    else:
      raise ImuBadMessage(raw_msg)

  def getMsgProcessed(self):
    msg = self.getMsg()
    return { 
        'ang_vel' : [float(msg.group('roll')),
                     float(msg.group('pitch')),
                     float(msg.group('yaw'))],
        'lin_acc' : [float(msg.group('x')),
                     float(msg.group('y')),
                     float(msg.group('z'))]
      }

  def getMsgRaw(self):
    if self.mode == 'p':
      return self.ser.readline()
    else:
      self.ser.write('g')
      return self.ser.readline()

if __name__ == '__main__':
  arduimu = ImuDriver()

