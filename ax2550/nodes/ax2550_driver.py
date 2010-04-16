#!/usr/bin/env python
# encoding: utf-8

"""
logerror.py - Contains a set of functions that facilitate special error 
logs that account for control code and Hardware Module special cases

Created by William Woodall on 2010-04-13.
"""
__author__ = "William Woodall"
__copyright__ = "Copyright (c) William Woodall"

###  Imports  ###

# ROS imports
import roslib; roslib.load_manifest('ax2550')
import rospy

# ROS msg and srv imports
from std_msgs.msg import String
from ax2550.srv import Move

# Python Libraries
from threading import Timer, Lock
from time import sleep

# pySerial
from serial import Serial

# Peer Libraries
from seriallistener import SerialListener
from logerror import logError


###  Classes  ###
class AX2550(object):
    """This class allows you to control an ax2550 motor controller"""
    def __init__(self, serial_port=None):
        """Function called after object instantiation"""
        # Get the serial port name
        self.serial_port = serial_port or '/dev/ttyUSB1'
        
        # Try to open and configure the serial port
        self.serial = Serial(self.serial_port)
        self.serial.timeout = 0.4
        self.serial.baud = "9600"
        self.serial.bytesize = 7
        self.serial.parity = "E"
        self.serial.stopbits = 1
        self.serial.close()
        self.serial.open()
        self.keep_alive_timer = None
        
        # Setup the lock to synchronize the setting of motor speeds
        self.speed_lock = Lock()
        self.serial_lock = Lock()
        
        # Set the motor speed to zero and start the dead man switch counter measure
        self.left_speed = 0
        self.right_speed = 0
        self.running = True
        self.sync()
        
        # Setup a Serial Listener
        self.serial_listener = SerialListener(self.serial)
        self.serial_listener.addHandler(self.isInRCMode, self.sync)
        self.serial_listener.listen()
        
        # Initialize ROS Node
        rospy.init_node('ax2550_driver', anonymous=True)
        
        # Subscribe to the /motor_control topic to listen for motor commands
        rospy.Subscriber('motor_control', String, self.controlCommandReceived)
        
        # Setup Publisher for publishing status related data to the /motor_control_status topic
        self.status_pub = rospy.Publisher('motor_control_status', String)
        
        # Register the Move service with the handleMove function
        self.move_srv = rospy.Service('move', Move, self.handleMove)
        
        # Register shutdown function
        rospy.on_shutdown(self.shutdown)
        
        # Handle ros srv requests
        rospy.spin()
        
    def handleMove(self, data):
        """Handles the Move srv requests"""
        self.move(data.speed, data.direction)
        return 0
        
    def controlCommandReceived(self, msg):
        """Handle's messages received on the /motor_control topic"""
        self.move(msg.speed, msg.direction)
        rospy.logdebug("Move command received on /motor_control topic: %s speed and %s direction" % (msg.speed, msg.direction))
        return 0
        
    def isInRCMode(self, msg):
        """Determines if a msg indicates that the motor controller is in RC mode"""
        if msg != '' and msg[0] == ':':
            # It is an RC message
            rospy.loginfo('Motor Controller appears to be in RC Mode, Syncing...')
            return True
        else:
            return False
    
    def sync(self, msg=None):
        """This function ensures that the motor controller is in serial mode"""
        rospy.loginfo("Syncing MC")
        listening = None
        if hasattr(self, 'serial_listener'):
            listening = self.serial_listener.isListening()
        if listening:
            self.serial_listener.stopListening()
        try:
            self.serial_lock.acquire()
            self.speed_lock.acquire()
            # First clean the buffers out
            sio = self.serial
            sio.flushInput()
            sio.flushOutput()
            sio.flush()
            # Reset the Motor Controller, incase it is in the Serial mode already
            sio.write('\r\n'+'%'+'rrrrrr\r\n')
            changing_modes = True
            line = ''
            token = sio.read(1)
            while changing_modes:
                line += token
                if token == '\r':
                    if line.strip() != '':
                        pass
                        # print line
                    line = ''
                    sio.write('\r')
                    token = sio.read(1)
                if token == 'O':
                    token = sio.read(1)
                    if token == 'K':
                        changing_modes = False
                else:
                    token = sio.read(1)
        finally:
            self.serial_lock.release()
            self.speed_lock.release()
            if listening:
                self.serial_listener.listen()
        rospy.loginfo('Motor Controller Synchronized')
    
    def shutdown(self):
        """Called when the server shutsdown"""
        self.running = False
        self.serial_listener.join()
        del self.serial_listener
        
    def start(self):
        """Called when Control Code Starts"""
        self.running = True
        self.keepAlive()
    
    def stop(self):
        """Called when Control Code Stops"""
        self.running = False
        if self.keep_alive_timer:
            self.keep_alive_timer.cancel()
    
    def disableKeepAlive(self):
        """Stops any running keep alive mechanism"""
        self.stop()
    
    def keepAlive(self):
        """This functions sends the latest motor speed to prevent the dead man 
            system from stopping the motors.
        """
        try:
            # Lock the speed lock
            self.speed_lock.acquire()
            # Resend the current motor speeds
            self.__setSpeed(self.left_speed, self.right_speed)
            # Release the speed lock
            self.speed_lock.release()
        except Exception as err:
            logError(sys.exc_info(), rospy.logerr, "Exception in keepAlive function: ")
        if self.running:
            self.keep_alive_timer = Timer(0.4, self.keepAlive)
            self.keep_alive_timer.start()
    
    def move(self, speed=0.0, direction=0.0):
        """Adjusts the motors based on the speed and direction you specify.
            
        Speed and Direction should be values between -1.0 and 1.0, inclusively.
        """
        #Validate the parameters
        if speed < -1.0 or speed > 1.0:
            error("Speed given to the move() function must be between -1.0 and 1.0 inclusively.")
            return
        if direction < -1.0 or direction > 1.0:
            error("Direction given to the move() function must be between -1.0 and 1.0 inclusively.")
            return
        #First calculate the speed of each motor then send the commands
        #Account for speed
        left_speed = speed
        right_speed = speed
        #Account for direction
        left_speed = right_speed + direction
        right_speed = right_speed - direction
        #Account for going over 1.0 or under -1.0
        if left_speed < -1.0:
            left_speed = -1.0
        if left_speed > 1.0:
            left_speed = 1.0
        if right_speed < -1.0:
            right_speed = -1.0
        if right_speed > 1.0:
            right_speed = 1.0
        #Send the commands
        self.setSpeeds(left=left_speed, right=right_speed)
    
    def setSpeeds(self, left=None, right=None):
        """Sets the speed of both motors"""
        # Lock the speed lock
        self.speed_lock.acquire()
        # Resend the current motor speeds
        if left != None:
            self.left_speed = left
        if right != None:
            self.right_speed = right
        self.__setSpeed(self.left_speed, self.right_speed)
        # Release the speed lock
        self.speed_lock.release()
    
    def __setSpeed(self, left, right):
        """Actually sends the appriate message to the motor"""
        # Form the commands
        #Left command
        left_command = "!"
        if left < 0:
            left_command += "A"
        else:
            left_command += "a"
        left = int(abs(left)*127)
        left_command += "%02X" % left
        #Right command
        right_command = "!"
        if right < 0:
            right_command += "B"
        else:
            right_command += "b"
        right = int(abs(right)*127)
        right_command += "%02X" % right
        # Lock the serial lock
        self.serial_lock.acquire()
        # Send the commands
        self.serial.write(left_command+'\r')
        self.serial.write(right_command+'\r')
        # Release the serial lock
        self.serial_lock.release()
    

###  If Main  ###
if __name__ == '__main__':
    AX2550()
