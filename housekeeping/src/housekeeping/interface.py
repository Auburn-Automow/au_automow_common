#!/usr/bin/env python
# encoding: utf-8

"""
interface.py - Contains the interface for controlling the housekeeping board.

Created by William Woodall on 2010-02-02.
"""
__author__ = "William Woodall"

###  Imports  ###

# Python libraries
import sys
from threading import Lock, Timer

# pySerial
from serial import Serial

# Peer libraries
from logerror import logError

###  Defines  ###

CMD_START       = '\x24' # $
CMD_STOP        = '\x0A' # LF

OP_START        = '\x61' # a
OP_STOP         = '\x66' # f
OP_DRIVE        = '\x67' # g
OP_SAFE_MODE    = '\x64' # d
OP_SENSOR       = '\x69' # i

SENSOR_ODOM     = '\x01' # b00000001
SENSOR_IR       = '\x02' # b00000010

START_CMD     = CMD_START+OP_START+CMD_STOP
STOP_CMD      = CMD_START+OP_STOP+CMD_STOP
SAFE_MODE_CMD = CMD_START+OP_SAFE_MODE+CMD_STOP

# Log system, this should be overriden with something like rospy.loginfo or rospy.logerr
#  I do this in an effort to remove all ros dependant code from this file
info = None
logerr = None

###  Classes  ###
class HousekeepingBoard(object):
    """This class allows you to control the housekeeping board"""
    def __init__(self, serial_port=None, hk_poll_rate=10):
        """Constructor"""
        # Use the passed parameters or the defaults
        self.serial_port = serial_port or "/dev/house_keeping"
        self.hk_poll_rate = 1.0/hk_poll_rate # Convert Hz to period
        # Create and setup the serial port
        self.serial = Serial()
        self.serial.port = self.serial_port
        self.serial.baudrate = 115200
        self.serial.open()
        
        # Setup some variables
        self.running = True
        
        self.serial_lock = Lock()
        
        self.onHousekeepingData = None
        self.hk_timer = None
        
        # Start the polling of the sensors
        self.pollHousekeeping()
    
    def pollHousekeeping(self):
        """Polls the HK Sensors on a regular period
            returns a list of the HK readings
        """
        if not self.running:
            return
        else:
            self.hk_timer = Timer(self.hk_poll_rate, self.pollHousekeeping) # Kick off the next timer
            self.hk_timer.start()
        pass
    

