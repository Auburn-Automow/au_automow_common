#!/usr/bin/env python

import os 

f = os.popen('pgrep rosout','r')
outp = f.readlines()
f.close()

if len(outp) > 0:
	f = os.popen('ps -p $(pgrep rosout) -o user','r')
	outp = f.readlines()
	f.close()
	outp = outp[1]
	outp = outp[:-1]
	print "ROS Started by " + outp
else:
	print "ROS Stopped"
