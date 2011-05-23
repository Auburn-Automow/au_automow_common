import numpy as np
from numpy.testing import *
import AutomowEKF as ekf 

class TestAutomowEKF():
    def test_modelUpdateF(self):        
        a = ekf.AutomowEKF.fromDefault()
        a.UpdateModel(np.array([1,1]),1)
        F = np.array([[ 1. ,  0. , -0. ,  0.5,  0.5,  0. ],
                      [ 0. ,  1. ,  1. ,  0. ,  0. ,  0. ],
                      [ 0. ,  0. ,  1. , -1. ,  1. ,  0. ],
                      [ 0. ,  0. ,  0. ,  1. ,  0. ,  0. ],
                      [ 0. ,  0. ,  0. ,  0. ,  1. ,  0. ],
                      [ 0. ,  0. ,  0. ,  0. ,  0. ,  1. ]])
        assert_array_equal(F,a.F)

    def test_modelUpdateG(self):
        a = ekf.AutomowEKF.fromDefault()
        a.UpdateModel(np.array([1,1]),1)
        G = np.array([[ 0.5,  0.5,  0. ,  0.5,  0.5,  0. ],
                      [ 0. ,  0. ,  0. ,  0.5,  0.5,  0. ],
                      [-1. ,  1. ,  1. , -1. ,  1. ,  0. ],
                      [ 0. ,  0. ,  0. ,  1. ,  0. ,  0. ],
                      [ 0. ,  0. ,  0. ,  0. ,  1. ,  0. ],
                      [ 0. ,  0. ,  0. ,  0. ,  0. ,  1. ]])
        assert_array_equal(G,a.G)

if __name__ == "__main__":
    run_module_suite()
