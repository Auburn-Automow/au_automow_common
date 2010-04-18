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
from rospy.rostime import Time

# ROS msg and srv imports
from std_msgs.msg import String
from ax2550.msg import Encoder
from ax2550.srv import Move

# Python Libraries
from threading import Timer, Lock
import time
from time import sleep
import sys

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
        self.encoder_timer = None
        self.encoder_rate = 0.05
        self.encoder_count = 0
        
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
        
        # Setup Publisher for publishing encoder data to the /motor_control_encoders topic
        self.encoders_pub = rospy.Publisher('motor_control_encoders', Encoder)
        
        # Setup Publisher for publishing status related data to the /motor_control_status topic
        self.status_pub = rospy.Publisher('motor_control_status', String)
        
        # Register the Move service with the handleMove function
        self.move_srv = rospy.Service('move', Move, self.handleMove)
        
        # Register shutdown function
        rospy.on_shutdown(self.shutdown)
        
        # Start polling the encoders
        self.pollEncoders()
        
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
    
    def decodeEncoderValue(self, data):
        """Decodes the Encoder Value"""
        # Determine if the value is negative or positive
        if data[0] in "01234567": # Positive
            fill_byte = "0"
        else: # Negative
            fill_byte = "F"
        # Fill the rest of the data with the filler byte
        while len(data) != 8:
            data = fill_byte+data
        # Now that the data has 8 Hex characters translate to decimal
        data = int(data, 16)
        if fill_byte == "F": # If negative subtract 2**32
            data -= 4294967296
        # Return the processed data
        return data
    
    def getHexData(self, msg, timeout=0.05):
        """Given a message to send the motor controller it will wait for the next Hex response"""
        if self.serial.isOpen():
            self.serial.write(msg) # Send the given request
            message = ""
            while True: # Data not received
                if not hasattr(self, 'serial_listener'): # Incase we are here during a ctrl-C
                    break
                message = self.serial_listener.grabNextUnhandledMessage(timeout) # Get next unhandled Message
                if message == None: # If None, timeout occured, drop data read
                    break
                if message[0] in 'ABCDEF0123456789': # If if starts with Hex data keep it
                    message = message.strip()
                    break
            return message
        else:
            return None
    
    def pollEncoders(self):
        """Polls the encoder on a period"""
        # Kick off the next polling iteration timer
        if self.running:
            self.encoder_timer = Timer(self.encoder_rate, self.pollEncoders)
            self.encoder_timer.start()
        else:
            return
        encoder_1 = None
        encoder_2 = None
        try:
            # Lock the speed lock
            self.speed_lock.acquire()
            # Lock the serial lock
            self.serial_lock.acquire()
            # Query encoder 1
            encoder_1 = self.getHexData("?Q4\r")
            # Query encoder 2
            encoder_2 = self.getHexData("?Q5\r")
            # Release the serial lock
            self.serial_lock.release()
            # Release the speed lock
            self.speed_lock.release()
            # Convert the encoder data to ints
            if encoder_1 != None:
                encoder_1 = self.decodeEncoderValue(encoder_1)
            else:
                encoder_1 = 0
            if encoder_2 != None:
                encoder_2 = self.decodeEncoderValue(encoder_2)
            else:
                encoder_2 = 0
            # Publish the encoder data
            message = Encoder(Time.from_seconds(time.time()), encoder_1, encoder_2)
            try:
                self.encoders_pub.publish(message)
            except:
                pass
        except Exception as err:
            logError(sys.exc_info(), rospy.logerr, "Exception while Querying the Encoders: ")
            self.encoder_timer.cancel()
    
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
