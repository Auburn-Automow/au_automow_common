#!/usr/bin/env python
import roslib; roslib.load_manifest('automow_tests')
import rospy

from nav_msgs.msg import Odometry
from magellan_dg14.msg import UTMFix

from Tkinter import *
from tkFileDialog import asksaveasfilename
import threading
import Queue

H = 800
W = 800

class FieldMap:
    def __init__(self):
        # Load Parameters
        self.gui_enabled = rospy.get_param("~gui_enabled",True)
        
        # Variables
        self.position_lock = threading.Lock()
        self.x = None
        self.y = None
        self.fix_type = None
        self.points = []
        self.current_position_needs_updating = False
        self.coordinates = []
        
        self.grabbing_point = False
        self.grabbing_thread = None
        
        self.new_point = False
        
        import os
        self.all_points_file = open(os.path.expanduser("~/all_points.csv"), "w")
        
        # GUI Stuff
        if self.gui_enabled:
            self.root = Tk()
            self.canvas = Canvas(width=W, height=H, bg='white')
            self.canvas.pack(expand=YES, fill=BOTH)
            self.base_station = self.canvas.create_polygon(H/2,W/2+8,H/2+8,W/2-8,H/2-8,W/2-8,width=2,fill='red')
            self.current_position = \
                self.canvas.create_oval(H/2+4,W/2+4,H/2-4,W/2-4,width=1,fill='')
            self.canvas.itemconfig(self.current_position, state=HIDDEN)
            self.status_text = self.canvas.create_text(W/2,H-20, text="No GPS Data received")
            self.drawGrid()
            self.canvas.bind("<Button-1>", self.grabPoint)
            self.canvas.bind("<Button-3>", self.saveData)
            self.position_queue = Queue.Queue()
            self.updateCanvas()
        
        # Topic Stuff
        rospy.Subscriber('/gps/odometry',Odometry,self.gpsOdometryCallback)
        rospy.Subscriber('/dg14_driver/utm_fix',UTMFix,self.gpsFixCallback)
        
        # Start mainloop
        if self.gui_enabled:
            threading.Thread(target=self.spin).start()
            self.root.mainloop()
            rospy.signal_shutdown("because")
        else:
            rospy.spin()
    
    def close(self):
        """docstring for close"""
        self.all_points_file.close()
    
    def drawGrid(self):
        for x in range(1,H/40):
            self.canvas.create_line(0, x*40, W, x*40, fill='black', dash=(2,2))
        for x in range(1,W/40):
            self.canvas.create_line(x*40, 0, x*40, H-40, fill='black', dash=(2,2))
    
    def saveData(self,data):
        try:
            file_name = asksaveasfilename()
            f = open(file_name, 'w')
            with self.position_lock:
                for x,y,fix in self.coordinates:
                    f.write(",".join((str(x),str(y),str(fix)))+"\n")
                f.close()
                self.coordinates = []
        except Exception as e:
            rospy.logerr(str(e))
    
    def grabPoint(self, data=None):
        """docstring for grabPoint"""
        if self.grabbing_point:
            self.grabbing_point = False
        else:
            self.grabbing_point = True
            self.grabbing_thread = threading.Thread(target=self._grabPoint)
            self.grabbing_thread.start()
    
    def _grabPoint(self):
        try:
            xs = []
            ys = []
            fix_type_ = None
            while self.grabbing_point:
                if rospy.is_shutdown():
                    return
                if not self.new_point:
                    continue
                with self.position_lock:
                    x = self.x
                    y = self.y
                    fix_type = self.fix_type
                    self.new_point = False
                xs.append(x)
                ys.append(y)
                fix_type_ = fix_type
                if None in [x,y,fix_type]:
                    rospy.logerr("Error, not enough information to collect a point.")
                    return
            if len(xs) == 0 or len(ys) == 0 or fix_type_ == None:
                rospy.logerr("Error, not enough information to collect a point.")
                return
            x = sum(xs)/len(xs)
            y = sum(ys)/len(ys)
            fix_type = fix_type_
            with self.position_lock:
                self.coordinates.append((x,y,fix_type))
            rospy.loginfo("Recording position: %f, %f, %f after averaging %f points"%(x,y,fix_type,len(xs)))
            self.position_queue.put((x,y,4,'red'))
        except Exception as e:
            rospy.logerr("Error in grabbing point: "+str(e))
    
    def gpsFixCallback(self, data):
        """docstring for gpsFixCallback"""
        with self.position_lock:
            self.fix_type = data.fix_type
    
    def updateCanvas(self):
        try:
            while True:
                (x,y,radius,color) = self.position_queue.get_nowait()
                x *= 4
                y *= -4
                self.canvas.create_oval(H/2+x+radius,W/2+y+radius,H/2+x-radius,W/2+y-radius,width=0,fill=color)
                self.canvas.delete(self.current_position)
                self.current_position = self.canvas.create_oval(H/2+x+4,W/2+y+4,H/2+x-4,W/2+y-4,width=1,fill='')
                self.canvas.itemconfig(self.current_position, state=NORMAL)
                if None not in [self.x, self.y, self.fix_type]:
                    self.canvas.delete(self.status_text)
                    text = ""
                    if self.grabbing_point:
                        text += "Averaging "
                    text += "Relative Easting: %f Northing: %f Fix Type: %f" % (self.y, self.x, self.fix_type)
                    self.status_text = self.canvas.create_text(W/2,H-20, text=text)
                self.canvas.update_idletasks()
        except Queue.Empty:
            pass
        
        self.root.after(20, self.updateCanvas) # 50 Hz
    
    def gpsOdometryCallback(self, data):
        """docstring for gpsOdometryCallback"""
        if self.gui_enabled:
            if self.x == None:
                self.dx = data.pose.pose.position.x
            else:
                self.dx = self.x - data.pose.pose.position.x
            if self.y == None:
                self.dy = data.pose.pose.position.y
            else:
                self.dy = self.y - data.pose.pose.position.y
        with self.position_lock:
            self.x = data.pose.pose.position.x
            self.y = data.pose.pose.position.y
        if self.gui_enabled:
            self.position_queue.put((self.x, self.y, 2, 'blue'))
            self.current_position_needs_updating = True
        self.all_points_file.write(str(data.pose.pose.position.x)+","+str(data.pose.pose.position.y)+"\n")
        self.new_point = True
    
    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('field_map')
    
    field_map = FieldMap()
    
    field_map.close()

if __name__ == '__main__':
    main()