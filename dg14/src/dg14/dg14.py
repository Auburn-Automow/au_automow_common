#!/usr/bin/env python
# encoding: utf-8

"""
dg14.py - Provides an interface

Created by William Woodall on 2010-04-13.
"""
__author__ = "William Woodall"
__copyright__ = "Copyright (c) William Woodall"

###  Imports  ###

# ROS imports
import roslib; roslib.load_manifest('dg14')
import rospy

# ROS msg and srv imports


# Python Libraries
import sys

# pySerial
from serial import Serial

# Peer Libraries
from seriallistener import SerialListener
from logerror import logError

###  Log Error  ###
# This should be overriden with something like rospy.logerr
info = lambda x: print x
logerr = lambda x: print x


###  Classes  ###
class DG14(object):
    """This class allows you to control an ax2550 motor controller"""
    def __init__(self, serial_port=None, rtk_mode=False):
        """Function called after object instantiation"""
        # Get the serial port name
        self.serial_port = serial_port or '/dev/ttyS0'
        self.rtk_mode = rtk_mode
        
        # Try to open and configure the serial port
        self.serial = Serial()
        self.serial.port = self.serial_port
        self.serial.timeout = 0.5
        self.serial.baud = "9600"
        self.serial.open()
        
        #GPS Parameters
        self.time = None
        self.longitude = None
        self.latitude = None
        self.altitude = None
        self.satseen = None
        self.speed = None
        self.heading = None
        self.pdop = None
        self.hdop = None
        self.vdop = None
        self.tdop = None
        self.quality = None
        self.age = None
        
        # Write the initialization commands for the GPS unit
        self.serial.write('$PASHQ,PRT*21\n')
        temp = self.serial.readline()
        if '$PASHR,PRT,A,5*56' not in temp:
            raise Exception("GPS is nonresponsive: %s" % temp)
        self.serial.write('$PASHQ,RID*28\n')
        if '$PASHR,RID,DG,DD04,TOPUB_LE_C__YXDR___I*17' not in temp:
            raise Exception("GPS is nonresponsive: %s" % temp)
        else:
            info("GPS is responsive")
        
        if self.rtk_mode:
            self.enterRTKMode()
    
    def queryPos(self):
        """takes the data from the GPS unit and stores it"""
        posdata = None
        posdata = self.serial.readline()
        if posdata != None:
            parsed_data = self.parseData(posdata)
            return posdata
    
    def parseData(self, data): 
        """Splits the sting of position from queryPos"""
        posarray = data.split(',')
        if posarray[0] == $SPASHR
            self.satseen = posarray[3]
            self.time = posarray[4]
            self.latitude = posarray[5]
            self.longitude = posarray[7]
            self.altitude = posarray[9]
            self.heading = posarray[11]
            self.speed = posarray[12]
            self.pdop = posarray[14]
            self.hdop = posarray[15]
            self.vdop = posarray[16]
            self.tdop = posarray[17]
        if posarray[0] == $GPGGA
            self.satseen = posarray[7]
            self.time = posarray[1]
            self.latitude = posarray[2]
            self.longitude = posarray[4]
            self.altitude = posarray[9]
            self.quality = posarray[6]
            self.age = posarray[13]
            self.hdop = posarray[8]
        return posarray
    
    def enterRTKMode(self):
        """Attempts to enter RTK Mode"""
        # Write initialization commands for RTCM mode
        try:
            self.serial.write('$PASHS,RST*20\n')
            temp = self.serial.readline()
            if '$PASHR,ACK*3D' not in temp:
                raise RuntimeError("Reset Command Not Acknowledged")
            
            self.serial.write('$PASHS,SPD,A,5*46\n')
            temp = self.serial.readline()
            if '$PASHR,ACK*3D' not in temp:
                raise RuntimeError("Speed A Command Not Acknowledged")
            
            self.serial.write('$PASHS,SPD,B,5*45\n')
            temp = self.serial.readline()
            if '$PASHR,ACK*3D' not in temp:
                raise RuntimeError("Speed B Command Not Acknowledged")
            
            self.serial.write('$PASHS,SMI,600,2*26\n')
            temp = self.serial.readline()
            if '$PASHR,ACK*3D' not in temp:
                raise RuntimeError("Smoothing Interval Command Not Acknowledged")
            
            self.serial.write('$PASHS,PEM,5*34\n')
            temp = self.serial.readline()
            if '$PASHR,ACK*3D' not in temp:
                raise RuntimeError("Position Elevation Mask Command Not Acknowledged")
            
            self.serial.write('$PASHS,RTC,REM,B*28\n')
            temp = self.serial.readline()
            if '$PASHR,ACK*3D' not in temp:
                raise RuntimeError("Remote Station Command Not Acknowledged")
            
            self.serial.write('$PASHS,RTC,AUT,Y*29\n')
            temp = self.serial.readline()
            if '$PASHR,ACK*3D' not in temp:
                raise RuntimeError("Enable Raw Data Command Not Acknowledged")
            
            self.serial.write('$PASHS,RTC,MAX,60*62\n')
            temp = self.serial.readline()
            if '$PASHR,ACK*3D' not in temp:
                raise RuntimeError("Max Age of Corrections Command Not Acknowledged")
            
            self.serial.write('$PASHS,NME,GGA,A,ON*1E\n')
            temp = self.serial.readline()
            if '$PASHR,ACK*3D' not in temp:
                raise RuntimeError("Enable GGA Messages Command Not Acknowledged")
            
            temp = self.serial.readline()
            temparray = temp.split(',')
            if temparray[6] == 2:
                pass
            elif temparry[6] == 1:
                self.serial.write('$PASHS,NME,GGA,A,OFF*1D\n')
                self.serial.write('$PASHS,NME,POS,A,ON*13\n')
                temp = self.serial.readline()
                if '$PASHR,ACK*3D' not in temp:
                    raise RuntimeError("Enable POS Messages Command Not Acknowledged")
        except RuntimeError as e:
            logerr("GPS is nonresponsive:\n" + str(e))
    

###  If Main  ###
if __name__ == '__main__':
    DG14()
