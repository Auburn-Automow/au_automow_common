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


if __name__ == "__main__":
    run_module_suite()
