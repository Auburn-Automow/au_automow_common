#!/usr/bin/env python
# encoding: utf-8

"""
interface.py - Contains the interface for the Magellan DG14 GPS.

Created by William Woodall on 2010-05-18.
"""
__author__ = "William Woodall"

###  Imports  ###

# Python libraries
import sys
from time import sleep

# pySerial
from serial import Serial

# Peer libraries
from logerror import logError
from seriallistener import SerialListener

###  Defines  ###

CMD_TERMINATOR = '\r\n'

CMD_POS_ON = '$PASHS,NME,POS,A,ON,0.2' + CMD_TERMINATOR
CMD_POS_OFF = '$PASHS,NME,POS,A,OFF,0.2' + CMD_TERMINATOR

CMD_SAT_ON = '$PASHS,NME,SAT,A,ON,0.2' + CMD_TERMINATOR
CMD_SAT_OFF = '$PASHS,NME,SAT,A,OFF,0.2' + CMD_TERMINATOR

CMD_UTM_ON = '$PASHS,NME,UTM,A,ON,0.2' + CMD_TERMINATOR
CMD_UTM_OFF = '$PASHS,NME,UTM,A,OFF,0.2' + CMD_TERMINATOR

###  Functions  ###

def floatOrNone(val):
    """Returns a float or None"""
    try:
        return float(val)
    except:
        return None

def intOrNone(val):
    """Returns an int or None"""
    try:
        return int(val)
    except:
        return None

###  Classes  ###
class DG14GPS(object):
    """This class allows you to interface with the DG14 GPS"""
    def __init__(self, serial_port=None):
        # Use the passed parameters or the defaults
        self.serial_port = serial_port or "/dev/gps"
        # Create and setup the serial port
        self.serial = Serial()
        self.serial.port = self.serial_port
        self.serial.baudrate = 115200
        self.serial.timeout = 2
        self.serial.open()
        
        self.running = False
        self.current_pos = None
        self.current_utm = None
        self.connected = False
        self.onPublishGPS = None
    
    def logerr(self, msg):
        """Prints the error msg to the stderr"""
        print >> sys.stderr, msg
    
    def info(self, msg):
        """Prints the msg to the stdout"""
        print msg
    
    def getData(self, timeout=0.25):
        """Waits for serial data and processes it if received with in the timeout period"""
        self.serial.timeout = timeout
        msg = self.serial.readline()
        if msg == None or msg == '':
            return False
        else:
            self.handleResponse(msg)
            return True
    
    def handleResponse(self, msg):
        """Handles incoming responses from the GPS"""
        msg = msg.split(',')
        if msg[0] != '$PASHR':
            return
        if msg[1] == "POS":
            if self.current_pos == None:
                self.current_pos = msg
        elif msg[1] == "UTM":
            if self.current_utm == None:
                self.current_utm = msg
        # If both pos and utm are set, process and publish
        if self.current_pos and self.current_utm:
            self.publishMessage()
            self.current_pos = None
            self.current_utm = None
    
    def publishMessage(self):
        """Publishes the two messages, pos an utm, as one message"""
        if self.onPublishGPS:
            # Parse out the data from POS
            try:
                msg = self.current_pos
                position_type           = \
                    intOrNone(msg[2])           # position type: 0 = autonomous 1 = position differentially corrected
                number_of_sats_pos      = \
                    intOrNone(msg[3])           # Number of satellites used in position computation (no leading zero)
                time_pos                = \
                    floatOrNone(msg[4])         # Current UTC time, (hhmmss), of position computation in hours, minutes and seconds
                latitude                = \
                    floatOrNone(msg[5])         # Latitude component of position in degrees, minutes, and fraction of minutes (ddmm.mmmm)
                latitude_sector         = \
                    msg[6]                      # Latitude sector: N = North, S = South
                longitude               = \
                    floatOrNone(msg[7])         # Longitude component of position in degrees, minutes, and fraction of minutes
                longitude_sector        = \
                    msg[8]                      # Longitude sector: E = East, W = West
                altitude               = \
                    floatOrNone(msg[9])         # Altitude in meters above WGS-84 reference ellipsoid. 
                                                # For 2-D position computation this item contains the altitude held fixed.
                                                # If f1 positive, “+” sign not mentioned
                site_id                 = \
                    msg[10]                     # Site ID
                heading                 = \
                    floatOrNone(msg[11])        # True track/true course over ground in degrees (000.00 to 359.99 degrees)
                speed                   = \
                    floatOrNone(msg[12])        # Speed over ground in knots
                velocity                = \
                    floatOrNone(msg[13])        # Vertical velocity in meters per second
                pdop                    = \
                    floatOrNone(msg[14])        # PDOP—position dilution of precision
                hdop                    = \
                    floatOrNone(msg[15])        # HDOP—horizontal dilution of precision
                vdop                    = \
                    floatOrNone(msg[16])        # VDOP—vertical dilution of precision
                tdop                    = \
                    floatOrNone(msg[17])        # TDOP—time dilution of precision
                firmware_version        = \
                    msg[18]                     # Firmware version ID
                
                # Parse out the data from UTM
                msg = self.current_utm
                time_utm                = \
                    floatOrNone(msg[2])         # UTC of position in hours, minutes, and decimal seconds (hhmmss.ss)
                utm_zone                = \
                    msg[3]                      # Zone number for the coordinates
                utm_easting             = \
                    floatOrNone(msg[4])         # East UTM coordinate (meters)
                utm_northing            = \
                    floatOrNone(msg[5])         # North UTM coordinate (meters)
                fix_mode                = \
                    intOrNone(msg[6])           # Position fix mode indicator. 
                                                # 1—Raw position 
                                                # 2—RTCM differential or CPD float 
                                                # 3—Carrier phase differential (CDP) fixed
                num_sats_used           = \
                    intOrNone(msg[7])           # Number of GPS satellites being used to compute positions
                hdop_utm                = \
                    floatOrNone(msg[8])         # Horizontal dilution of precision (HDOP)
                antenna_height          = \
                    floatOrNone(msg[9])         # Antenna height (meters)
                geoidal_sep             = \
                    floatOrNone(msg[10])        # Geoidal separation in meters
                geoidal_units           = \
                    msg[11]                     # Geoidal separation units (meters)
                age_of_dif_correction   = \
                    intOrNone(msg[12])          # Age of differential corrections
                dif_station_id          = \
                    msg[13]                     # Differential reference station ID
                
                if time_pos != time_utm:
                    self.logerr("Clock skew detected in POS and UTM messages:\nPOS - Time: %f, Message: %s\nUTM - Time: %f, Message: %s\n" % \
                                (time_pos, ','.join(self.current_pos), time_utm, ','.join(self.current_utm)))
                    return
                # Assemble and publish the message
                result = []
                result.append(time_pos)
                if latitude_sector == "S": # If southern hemisphere, it needs to be negative
                    latitude *= -1
                result.append(latitude)
                if longitude_sector == "W": # If West of Prime meridian, it needs to be negative
                    longitude *= -1
                result.append(longitude)
                result.append(altitude)
                result.append(utm_easting)
                result.append(utm_northing)
                result.append(utm_zone)
                result.append(heading)
                result.append(hdop)
                result.append(vdop)
                result.append(0) # err_horizontal
                result.append(0) # err_vertical
                # Fix Quality
                if longitude == None:
                    result.append(0) # No Fix
                elif position_type == 0:
                    result.append(1) # GPS Fix
                elif position_type == 1:
                    result.append(2) # DGPS Fix
                else:
                    result.append(0) # Fallback default
                result.append(number_of_sats_pos)
                self.onPublishGPS(result)
            except Exception:
                logError(sys.exc_info(), self.logerr, "Exception parsing POS or UTM data: ")
    
    def start(self):
        """Starts the GPS into polling mode if it isn't"""
        # Send stop commands
        self.serial.write(CMD_POS_OFF)
        self.serial.write(CMD_UTM_OFF)
        self.serial.write("$PASHQ,PRT\r\n")
        self.serial.flushInput()
        self.serial.write(CMD_POS_ON)
        self.serial.flush()
        if "$PASHR,ACK" not in self.serial.readline():
            return False
        self.serial.flushInput()
        self.serial.write(CMD_UTM_ON)
        if "$PASHR,ACK" in self.serial.readline():
            return False
        return True
    
    def shutdown(self):
        """Called to shutdown the threads and close the seiral port"""
        self.running = False
        self.serial.close()
    

