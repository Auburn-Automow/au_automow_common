#!/usr/bin/env python

"""
GIU to survey a set of field coordinates.
"""

import roslib; roslib.load_manifest('automow_maps')
import rospy

from magellan_dg14.msg import UTMFix

import sys, os
from yaml import dump

from math import pi, degrees

from PySide import QtCore, QtGui

class RenderArea(QtGui.QWidget):
    """Render area for rendering the robot"""
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        
        rospy.init_node("field_surveyor")
        rospy.Subscriber("/magellan_dg14/utm_fix", UTMFix, self.onUTMFixMsg)

        self.easting_offset = rospy.get_param('/magellan_dg14/easting_offset')
        self.northing_offset = rospy.get_param('/magellan_dg14/northing_offset')

        newFont = self.font()
        newFont.setPixelSize(12)
        self.setFont(newFont)
        
        self.gps_points = []
        self.current_points = []
        self.averaged_points = []
        self.current_utm = ['nan', 'nan', 'none']
        self.state = 'not_recording'

        fontMetrics = QtGui.QFontMetrics(newFont)
        self.xBoundingRect = fontMetrics.boundingRect(self.tr("x"))
        self.yBoundingRect = fontMetrics.boundingRect(self.tr("y"))

    def mouseReleaseEvent(self, event):
        if self.state == 'recording':
            # Stop the recording
            self.state = 'not recording'
            if event.button() == QtCore.Qt.MouseButton.LeftButton:
                # Average the points
                self.average_points()
        else:
            if event.button() == QtCore.Qt.MouseButton.LeftButton:
                # Start recording
                self.current_points = []
                self.state = 'recording'
            else:
                # Save out to a file
                self.save_to_file()
        self.update()
    
    def average_points(self):
        easting_sum = 0
        northing_sum = 0
        fix_type_sum = 0
        num = float(len(self.current_points))
        if num == 0:
            return
        for point in self.current_points:
            easting_sum += point[0]
            northing_sum += point[1]
            fix_type_sum += point[2]
        averaged_point = {'easting': easting_sum/num,
                          'northing': northing_sum/num,
                          'fix_type': fix_type_sum/num}
        self.averaged_points.append(averaged_point)

    def save_to_file(self):
        file_name = QtGui.QFileDialog.getSaveFileName(self, 'Save survey points to...')
        if len(self.averaged_points) == 0:
            return
        yaml_out = dump(self.averaged_points)
        if file_name:
            open(file_name[0], 'w').write(yaml_out)

    def paintEvent(self, event):
        """Handles painting the render area"""
        painter = QtGui.QPainter()
        painter.begin(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(event.rect(), QtGui.QBrush(QtCore.Qt.black))
        
        painter.save()
        # Correct origin location and rotation
        painter.translate(self.size().width()/2.0, self.size().height()/2.0)
        self.drawReticle(painter)
        self.drawGPSPoints(painter)
        painter.restore()

        self.drawMessage(painter)

        painter.save()
        self.drawGlobalCoordinates(painter)
        painter.restore()
        
        painter.end()
    
    def onUTMFixMsg(self, msg):
        easting = msg.easting - self.easting_offset
        northing = msg.northing - self.northing_offset
        self.gps_points.append(QtCore.QPointF(easting*10, -northing*10))
        self.current_utm = [easting, northing, msg.fix_type]
        if self.state == 'recording':
            entry = (easting, northing, msg.fix_type)
            self.current_points.append(entry)
        self.update()

    def drawGPSPoints(self, painter):
        pen = QtGui.QPen(QtCore.Qt.red, 3)
        painter.setPen(pen)
        for point in self.gps_points:
            painter.drawPoint(point)
        pen = QtGui.QPen(QtCore.Qt.blue, 5)
        painter.setPen(pen)
        for point in self.averaged_points:
            painter.drawPoint(QtCore.QPointF(point['easting']*10, -point['northing']*10))
    
    def drawMessage(self, painter):
        """Draws the instructions"""
        painter.setPen(QtCore.Qt.white)
        msg = ""
        if self.state == 'recording':
            msg = "Left click to save this average, Right click to cancel it"
        else:
            msg = "Left click to begin averaging, Right click to save to file."
        painter.drawText(self.size().width() - 400, self.size().height() - 10, msg)
        msg = "UTM easting: %s, northing: %s, fix_type: %s"%(tuple(self.current_utm))
        painter.drawText(self.size().width() - 400, self.size().height() - 25, msg)
        if self.state == 'recording':
            msg = "Number of points averaged: %i"%len(self.current_points)
        else:
            msg = "Number of points surveyed: %i, number of gps positions received: %i"%(len(self.averaged_points), len(self.gps_points))
        painter.drawText(self.size().width() - 400, self.size().height() - 40, msg)
    
    def drawReticle(self, painter):
        """Draws a little reticle at the origin"""
        painter.setPen(QtCore.Qt.white)
        w = self.size().width()
        h = self.size().height()
        offset_x = 0
        offset_y = 0
        while offset_x < w or offset_y < h:
            painter.drawLine(-w/2.0, offset_y, w/2.0, offset_y)
            painter.drawLine(-w/2.0, -offset_y, w/2.0, -offset_y)
            offset_y += 100
            painter.drawLine(offset_x, -h/2.0, offset_x, h/2.0)
            painter.drawLine(-offset_x, -h/2.0, -offset_x, h/2.0)
            offset_x += 100
    
    def drawGlobalCoordinates(self, painter):
        offset_x = 10
        offset_y = self.size().height() - 10
        painter.translate(offset_x, offset_y)
        
        painter.setPen(QtCore.Qt.red)
        
        painter.drawLine(0, 0, 50, 0)
        painter.drawLine(48, -2, 50, 0)
        painter.drawLine(48, 2, 50, 0)
        painter.save()
        painter.translate(48, -12)
        text_doc = QtGui.QTextDocument()
        text_doc.setHtml('<p style="color: red; font: 12px;">E</p>')
        text_doc.drawContents(painter)
        painter.restore()
        
        painter.setPen(QtCore.Qt.green)
        
        painter.drawLine(0, 0, 0, -50)
        painter.drawLine(-2, -48, 0, -50)
        painter.drawLine(2, -48, 0, -50)
        painter.save()
        painter.translate(-8, -70)
        text_doc = QtGui.QTextDocument()
        text_doc.setHtml('<p style="color: rgb(0,247,0); font: 12px;">N</p>')
        text_doc.drawContents(painter)
        painter.restore()

def main():
    app = QtGui.QApplication(sys.argv)
    ra = RenderArea()
    ra.resize(800, 600)
    ra.setWindowTitle("Field Surveyor")
    ra.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
