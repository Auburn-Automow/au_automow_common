import roslib; roslib.load_manifest('automow_ekf')

import numpy as np
from numpy.testing import *
from automow_ekf import AutomowEKF as ekf 
import scipy.io as sio

class TestAutomowEKF():
    def test_modelUpdate(self):        
        for x in range(1,6):
            filename = "./test_data/UpdateModelTest" + str(x) + ".mat" 
            data = sio.loadmat(filename,struct_as_record=True)
            test_filter = ekf(
                np.squeeze(data['x_hat_i']),
                data['P'],
                data['Q'],
                data['R_gps'],
                data['R_imu'])
            test_filter.updateModel(np.squeeze(data['u']),np.squeeze(data['dt']))
            assert_array_equal(data['F'],test_filter.F)
            assert_array_equal(data['G'],test_filter.G)

    def test_TimeUpdate(self):
        for x in range(1,6):
            filename = "./test_data/TimeUpdateTest" + str(x) + ".mat"
            data = sio.loadmat(filename,struct_as_record=True)
            test_filter = ekf(
                np.squeeze(data['x_hat_i']),
                data['P'],
                data['Q'],
                data['R_gps'],
                data['R_imu'])

    def test_WrapToPi(self):
        filename = "./test_data/WrapToPiTest.mat"
        data = sio.loadmat(filename,struct_as_record=True)
        data = np.hstack((data['input'],data['output'].T))
        for (inp,out) in data:
            assert_almost_equal(ekf.wrapToPi(inp),out,decimal=12)


if __name__ == "__main__":
    run_module_suite()
