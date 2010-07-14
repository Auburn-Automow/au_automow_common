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

CMD_TERMINATOR      = "\r"

CMD_RESET           = "R"

CMD_LEFT_CUTTER     = "S1 "
CMD_RIGHT_CUTTER    = "S2 "

CMD_GET_ALL         = "G0"

###  Classes  ###
class HousekeepingBoard(object):
    """This class allows you to control the housekeeping board"""
    def __init__(self, serial_port=None, hk_poll_rate=5):
        """Constructor"""
        # Use the passed parameters or the defaults
        self.serial_port = serial_port or "/dev/house_keeping"
        self.hk_poll_rate = 1.0/hk_poll_rate # Convert Hz to period
        # Create and setup the serial port
        self.serial = Serial()
        self.serial.port = self.serial_port
        self.serial.baudrate = 115200
        self.serial.timeout = 0.25
        self.serial.open()
        
        # Reset the Housekeeping Board
        self.serial.write(CMD_RESET+CMD_TERMINATOR)
        
        # Setup some variables
        self.running = True
        
        self.serial_lock = Lock()
        
        self.onHousekeepingData = None
        self.hk_timer = None
    
    def logerr(self, msg):
        """Prints the error msg to the stderr"""
        print >> sys.stderr, msg
    
    def info(self, msg):
        """Prints the msg to the stdout"""
        print msg
    
    def pollHousekeeping(self):
        """Polls the HK Sensors on a regular period
            returns a list of the HK readings
        """
        if not self.running:
            return
        else:
            self.hk_timer = Timer(self.hk_poll_rate, self.pollHousekeeping) # Kick off the next timer
            self.hk_timer.start()
        # Make sure the serial port is open
        if not self.serial.isOpen():
            print("Serial Port not open, dieing...")
            self.running = False
            return
        data = ""
        # Get the Data
        try:
            # Prevent Cross talk, secure the serial port
            self.serial_lock.acquire()
            # Send the Get all command
            if self.serial.isOpen():
                self.serial.write(CMD_GET_ALL+CMD_TERMINATOR)
            else:
                return
            # While the response is not received wait for serial input
            count = 0
            iteration_limit = 5
            while self.running and count != iteration_limit:
                if data and len(data) != 0 and data[0] == "G":
                    break
                if self.serial.isOpen():
                    data = self.serial.readline()
                count += 1
            if count == iteration_limit:
                return
        except Exception as err:
            logError(sys.exc_info(), self.logerr, "Exception while wating on Housekeeping data response: ")
        finally:
            self.serial_lock.release()
        try:
            # Parse the data
            if not data or data == "" or data[0] != "G":
                return
            data = data.split('\t')
            if len(data) != 14:
                logerr("Malformed housekeeping data")
                return
            if self.onHousekeepingData:
                self.onHousekeepingData(data)
        except Exception as err:
            logError(sys.exc_info(), self.logerr, "Exception while parsing Housekeeping data: ")
    
    def cutterControl(self, left=False, right=False):
        """Controls the cutters"""
        result = False
        if left:
            left = '1'
        else:
            left = '0'
        if right:
            right = '1'
        else:
            right = '0'
        # Control cutters
        self.serial_lock.acquire()
        try:
            self.serial.write(CMD_LEFT_CUTTER+left+CMD_TERMINATOR)
            self.serial.write(CMD_RIGHT_CUTTER+right+CMD_TERMINATOR)
            result = True
        except:
            logError(sys.exc_info(), self.logerr, "Exception while controlling the cutting motors: ")
        finally:
            self.serial_lock.release()
        return result
    
    def shutdown(self):
        """Called to shutdown the threads and close the seiral port"""
        if self.hk_timer:
            self.hk_timer.cancel()
        self.running = False
        self.serial.close()
    

