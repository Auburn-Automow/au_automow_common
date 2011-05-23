import roslib; roslib.load_manifest('automow_ekf')

import numpy as np
from numpy.testing import *
from automow_ekf import AutomowEKF as ekf 
import scipy.io as sio

class TestAutomowEKF():
    def test_modelUpdate(self):
        """ Test the model update against the equivalent MATLAB function """
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
        """ Test the time update against MATLAB data """
        filename = "./test_data/TimeUpdateTest.mat"
        data = sio.loadmat(filename,struct_as_record=True)
        test_filter = ekf(
            np.squeeze(data['x_hat_i']),
            data['P_i'],
            data['Q'],
            data['R_gps'],
            data['R_imu'])
        for x in range(len(data['u'])):
            (v,w) = test_filter.timeUpdate(np.squeeze(data['u'][x]), \
                    np.squeeze(data['time'][x]))
            assert_almost_equal(v,np.squeeze(data['v'][0,x]),decimal=10, \
                    err_msg = "v, On Iteration: " + str(x+1))
            assert_almost_equal(w,np.squeeze(data['w'][0,x]),decimal=10, \
                    err_msg = "w, On Iteration: " + str(x+1))
            assert_array_almost_equal(test_filter.x_hat, \
                    np.squeeze(data['x_hat'][x]),decimal=10, \
                    err_msg = "x_hat, On Iteration: " + str(x+1))
            assert_array_almost_equal(data['P'][x],test_filter.P, \
                    decimal=10, err_msg = "P, On Iteration: " + str(x+1))

    def test_measurementUpdateGPS(self):
        """ Test the GPS measurement update against MATLAB data 
        without using the adaptive portion of the filter """
        filename = "./test_data/measurementUpdateGPSTest.mat"
        data = sio.loadmat(filename,struct_as_record=True)
        test_filter = ekf(
            np.squeeze(data['x_hat_i']),
            data['P_i'],
            data['Q'],
            data['R_gps'],
            data['R_imu'])

        test_filter.timeUpdate(np.squeeze(data['u']), np.squeeze(data['time']))
        
        for x in range(data['numCases']):
            inp = data['y_gps'][x]
            (inv, S, K) = test_filter.measurementUpdateGPS( \
                    inp,np.squeeze(data['R_gps']))

            assert_array_almost_equal(inv,np.squeeze(data['innovation'][x]),
                    err_msg = "innvation, on iteration: " + str(x+1))
            assert_array_almost_equal(S,data['S'][x], \
                    err_msg = "error covariance, on iteration: " + str(x+1))
            assert_array_almost_equal(K,data['K'][x], \
                    err_msg = "Kalman Gain, on iteration: " + str(x+1))
            assert_array_almost_equal(test_filter.x_hat,data['x_hat'][x], \
                    err_msg = "x_hat, on iteration: " + str(x+1))
            assert_array_almost_equal(test_filter.P,data['P'][x], \
                    err_msg = "P, on iteration: " + str(x+1))

 


    def test_measurementUpdateAHRS(self):
        """ Test the AHRS measurement update against MATLAB data """


    def test_WrapToPi(self):
        """ Testcase for WrapToPi against MATLAB's function """

        filename = "./test_data/WrapToPiTest.mat"
        data = sio.loadmat(filename,struct_as_record=True)
        data = np.hstack((data['input'],data['output'].T))
        for (inp,out) in data:
            assert_almost_equal(ekf.wrapToPi(inp),out,decimal=12)


if __name__ == "__main__":
    run_module_suite()
