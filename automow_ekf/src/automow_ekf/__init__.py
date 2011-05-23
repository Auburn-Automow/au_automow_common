import numpy

class AutomowEKF():
    """docstring for AutomowEKF"""
    def __init__(self):
        print "In constructor"
    
    def timeUpdate(self, left_wheel, right_wheel, current_time):
        """docstring for timeUpdate"""
        print left_wheel, right_wheel, current_time
    
    def measurementUpdateAHRS(self, measurement, covariance):
        """docstring for measurementUpdateAHRS"""
        print measurement, covariance
    
