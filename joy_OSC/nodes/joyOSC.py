import sys
import threading
import OSC
import traceback
from time import sleep

import roslib; roslib.load_manifest('joy_OSC')
import rospy
from joy.msg import Joy

class joyOSC:
  def __init__(self,local):
    self.pub = rospy.Publisher("joy_OSC",Joy)
    self.local = local
    self.driveEnable = False 
    self.oscServ = OSC.ThreadingOSCServer(local)
    
    self.oscServ.addDefaultHandlers()
    self.oscServ.addMsgHandler("/1/xy1", self.xyHandler)
    self.oscServ.addMsgHandler("/1/xy1/z", self.zHandler) 
    self.th_oscServ = threading.Thread(target=self.oscServ.serve_forever)
    self.th_oscServ.start()

  def xyHandler(self, addr=None, tags=None, data=None, source=None):
    # Data comes in as a tuple [x,y]
    print "xyHandler reached"
    print type(data)
    if not self.driveEnable:
      return

    x = (data[0] * 2.0) - 1
    y = (data[1] * 2.0) - 1

    msg = Joy()
    msg.axes.append(x)
    msg.axes.append(y)
    self.pub.publish(msg)

  def zHandler(self, addr, tags, data, source):
    print "zHandler reached"
    if data[0] == 1.0:
      self.driveEnable = True
    else:
      self.xyHandler(data=[0.5,0.5])
      self.driveEnable = False
    
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
