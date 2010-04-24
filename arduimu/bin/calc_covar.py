import roslib; roslib.load_manifest('arduimu')
import rospy

from hw_interface.arduimu_driver import ImuDriver, ImuBadMessage
import numpy as np

def main():
  imu = ImuDriver()
  imu.setMode('p')
  lin_values = []
  ang_values = []
  for x in range(0, 500):
    result = imu.getMsgProcessed()
    lin_values.append(result['lin_acc'])
    ang_values.append(result['ang_vel'])
  
  ang = np.array(ang_values)
  ang_std = ang.std(axis=0)
  ang_covar = np.square(ang_std)
  lin = np.array(lin_values)
  lin_std = lin.std(axis=0)
  lin_covar = np.square(lin_std)

  print "ang: " + str(ang_covar)
  print "lin: " + str(lin_covar)

if __name__ == "__main__":
  main()
