import sys
import threading
import OSC
import traceback
from time import sleep
import math

import roslib; roslib.load_manifest('joy_OSC')
import rospy
from joy.msg import Joy

class joyOSC:
        dampSpeed = 5
        xrot = 0
        zrot = 0

        def __init__(self,local):
                self.pub = rospy.Publisher("joy_OSC",Joy)
                self.local = local
                self.driveEnable = False 
                self.accEnable = False
                self.accToggle = False
                self.oscServ = OSC.ThreadingOSCServer(local)

                self.oscServ.addDefaultHandlers()
                self.oscServ.addMsgHandler("/1/xy1", self.xyHandler)
                self.oscServ.addMsgHandler("/1/xy1/z", self.zHandler) 
                self.oscServ.addMsgHandler("/accxyz", self.accHandler)
                self.oscServ.addMsgHandler("/1/toggle1", self.toggleHandler)
                self.oscServ.addMsgHandler("/1/push1",self.deadmanHandler)
                self.th_oscServ = threading.Thread(target=self.oscServ.serve_forever)
                self.th_oscServ.start()

        def xyHandler(self, addr=None, tags=None, data=None, source=None):
                if not self.driveEnable:
                        return

                x = (data[0] * 2.0) - 1
                y = (data[1] * 2.0) - 1

                msg = Joy()
                msg.axes.append(x)
                msg.axes.append(y)
                self.pub.publish(msg)

        def zHandler(self, addr, tags, data, source):
                if data[0] == 1.0:
                        self.driveEnable = True
                else:
                        self.xyHandler(data=[0.5,0.5])
                        self.driveEnable = False

        def toggleHandler(self, addr, tags, data, source):
                if data[0] == 1.0:
                        self.accToggle = True
                else:
                        self.accToggle = False
                        self.driveEnable = True
                        self.xyHandler(data = [0.5,0.5])
                        self.driveEnable = False

        def deadmanHandler(self, addr, tags, data, source):
                if data[0] == 1.0 and self.accToggle:
                        self.accEnable = True
                else:
                        self.accEnable = False
                        self.driveEnable=True
                        self.xyHandler(data = [0.5,0.5])
                        self.driveEnable = False

        def accHandler(self, addr=None, tags=None, data=None, source=None):
                if not self.accEnable:
                        return
                xrot_targ = data[0]*90
                zrot_targ = data[1]*-90
                orientation = data[2]
                if(xrot_targ > self.xrot):
                        self.xrot = self.xrot+((xrot_targ-self.xrot) / self.dampSpeed)
                else:
                        self.xrot = self.xrot-((self.xrot-xrot_targ) / self.dampSpeed)
                if(zrot_targ > self.zrot):
                        self.zrot = self.zrot+((zrot_targ-self.zrot) / self.dampSpeed)
                else:
                        self.zrot = self.zrot-((self.zrot-zrot_targ) / self.dampSpeed)
                x = math.sin(math.radians(self.xrot))
                y = math.sin(math.radians(self.zrot))

                msg = Joy()
                msg.axes.append(x)
                msg.axes.append(y)
                self.pub.publish(msg)


if __name__ == '__main__':
        osc = None
        try:
                local = '',9000
                rospy.init_node('joy_OSC', log_level=rospy.DEBUG)
                osc = joyOSC(local)
                rospy.spin()
        except:
                traceback.print_exc(file=sys.stdout)
        finally:
                if osc:
                        osc.oscServ.close()
                        osc.th_oscServ.join()
