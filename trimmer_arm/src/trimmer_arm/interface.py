#!/usr/bin/env python
# encoding: utf-8

"""
interface.py - Contains the interface for controlling the trimmer arm board.

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


###  Classes  ###
class TrimmerArmBoard(object):
    """This class allows you to control the trimmer arm board"""
    def __init__(self, serial_port=None):
        """Constructor"""
        # Use the passed parameters or the defaults
        self.serial_port = serial_port or "/dev/trimmer_arm"
        # Create and setup the serial port
        self.serial = Serial()
        self.serial.port = self.serial_port
        self.serial.baudrate = 9600
        self.serial.timeout = 0.25
        self.serial.open()
        
        # Setup log
        self.log = open("/tmp/trimmer_arm.log", "w+")
        
        # Setup some variables
        self.running = True
    
    def logerr(self, msg):
        """Prints the error msg to the stderr"""
        print >> sys.stderr, msg
    
    def info(self, msg):
        """Prints the msg to the stdout"""
        print msg
    
    def logSerialData(self):
        """Logs some serial data"""
        if not self.serial.isOpen():
            return
        data = self.serial.readline()
        if data and data != "":
            self.info(data)
            self.log.write(data)
    
    def shutdown(self):
        """Called to shutdown the threads and close the seiral port"""
        self.running = False
        self.serial.close()
        self.log.close()
    

