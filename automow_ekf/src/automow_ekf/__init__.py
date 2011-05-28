import numpy as np
import threading

def wrapToPi(angle):
    """
    Wrap a given angle in radians to the range -pi to pi.

    @param angle : The angle to be wrapped
    @param type angle : float
    @return : Wrapped angle
    @rtype : float
    """
    return np.mod(angle+np.pi,2.0*np.pi)-np.pi

class AutomowEKF:
    __nx = 7                    # Number of States in the Kalman Filter        
    __ny_gps = 2                # Number of measurements from the GPS
    __ny_imu = 1                # Number of measurements from the IMU
    __nu = 2                    # Number of inputs
    __prev_time = 0
    __dt = np.double

    C_gps = np.array([[1,0,0,0,0,0,0],
                      [0,1,0,0,0,0,0]],dtype=__dt)
    C_imu = np.array([0,0,1,0,0,0,1],dtype=__dt)

    def __init__(self,
            x_hat_i, 
            P_i,
            Q,
            R_gps,
            R_imu):
        """
        Initialize the Kalman Filter with a set of input arguments

        @param x_hat_i : The initial state of the Kalman Estimator
        @param type x_hat_i : (7,) numpy.array, dtype=np.double
        @param P_i : The initial covariance matrix of the Kalman Estimator
        @param type P_i : (7,7) numpy.array, dtype=np.double
        @param Q : The process noise covariance of the system
        @param type Q : (7,7) numpy.array, dtype=np.double
        @param R_gps : The GPS measurement noise covariance 
        @param type R_gps : (2,2) numpy.array, dtype=np.double 
        @param R_imu : The AHRS measurement noise covariance
        @param type R_imu : (1,1) numpy.array, dtype=np.double
        """
        self.state_lock = threading.Lock()
        with self.state_lock:
            self.x_hat = x_hat_i
        self.P = P_i
        self.Q = Q
        self.R_gps = R_gps
        self.R_imu = R_imu
        
        self.F = np.zeros((self.__nx,self.__nx),dtype=self.__dt)
        self.G = np.zeros((self.__nx,self.__nx),dtype=self.__dt)

    @classmethod
    def fromDefault(cls):
        """
        Initialize the Kalman Filter with a set of default arguments
        """
        x_hat_i = np.array([0,0,0,0.159,0.159,0.5461,0],dtype=cls.__dt)
        P_i = np.diag(np.array([100,100,100,1e-3,1e-3,1e-3,1e-3],dtype=cls.__dt))
        Q = np.diag(np.array([0.2,0.2,0,0,0,0,0],dtype=cls.__dt))
        R_gps = np.eye(2,dtype=cls.__dt) * 0.1
        R_imu = np.eye(1,dtype=cls.__dt) * 0.012
        return cls(x_hat_i,P_i,Q,R_gps,R_imu)

    def updateModel(self,u,dt):
        """
        Update the process and process noise matricies of the model
        
        @param u : The current i
        @param type u : (2,) numpy.array, dtype=np.double
        @param dt : The time delta from the previous time update
        @param type dt : np.float
        """
        self.F = np.eye(self.__nx,dtype=self.__dt)
        self.F[0,2] = -0.5 * dt \
                * (self.x_hat[3] * u[0] + self.x_hat[4] * u[1]) \
                * np.sin(self.x_hat[2])
        self.F[0,3] = 0.5 * dt * u[0] * np.cos(self.x_hat[2])
        self.F[0,4] = 0.5 * dt * u[1] * np.cos(self.x_hat[2])
        self.F[1,2] = 0.5 * dt \
                * (self.x_hat[3] * u[0] + self.x_hat[4] * u[1]) \
                * np.cos(self.x_hat[2])
        self.F[1,3] = 0.5 * dt * u[0] * np.sin(self.x_hat[2])
        self.F[1,4] = 0.5 * dt * u[1] * np.sin(self.x_hat[2])
        self.F[2,3] = -1.0 * dt * u[0]/self.x_hat[5] 
        self.F[2,4] = dt * u[1]/self.x_hat[5]
        self.F[2,5] = dt \
                * (self.x_hat[3] * u[0] - self.x_hat[4] * u[1]) \
                / np.power(self.x_hat[5],2);
        
        self.G = np.zeros((self.__nx,self.__nx),dtype=self.__dt)
        self.G[0,0] = 0.5 * dt * self.x_hat[3] * np.cos(self.x_hat[2])
        self.G[0,1] = 0.5 * dt * self.x_hat[4] * np.cos(self.x_hat[2])
        self.G[0,3] = 0.5 * dt * u[0] * np.cos(self.x_hat[2])
        self.G[0,4] = 0.5 * dt * u[1] * np.cos(self.x_hat[2])
        self.G[1,0] = 0.5 * dt * self.x_hat[3] * np.sin(self.x_hat[2])
        self.G[1,1] = 0.5 * dt * self.x_hat[4] * np.sin(self.x_hat[2])
        self.G[1,3] = 0.5 * dt * u[0] * np.cos(self.x_hat[2])
        self.G[1,4] = 0.5 * dt * u[1] * np.cos(self.x_hat[2])
        self.G[2,0] = -1.0 * dt * self.x_hat[3]/self.x_hat[5]
        self.G[2,1] = dt * self.x_hat[4]/self.x_hat[5]
        self.G[2,2] = dt 
        self.G[2,3] = -1.0 * dt * self.x_hat[3]/self.x_hat[5] 
        self.G[2,4] = dt * self.x_hat[4]/self.x_hat[5] 
        self.G[3,3] = dt 
        self.G[4,4] = dt
        self.G[5,5] = dt
        self.G[6,6] = dt
        return
    
    def timeUpdate(self,u,time):
        dt = time - self.__prev_time
        self.__prev_time = time
        self.updateModel(u,dt)
        
        v = self.x_hat[4]/2.0 * u[1] + self.x_hat[3]/2.0 * u[0]
        w = self.x_hat[4]/self.x_hat[5] * u[1] - \
                self.x_hat[3]/self.x_hat[5] * u[0]
        with self.state_lock:
            self.x_hat[0] += dt * v * np.cos(self.x_hat[2] + dt * w/2.0)
            self.x_hat[1] += dt * v * np.sin(self.x_hat[2] + dt * w/2.0)
            self.x_hat[2] += dt * w
            self.x_hat[2] = wrapToPi(self.x_hat[2])
        self.P = np.dot(self.F,np.dot(self.P,self.F.T)) \
                + np.dot(self.G,np.dot(self.Q,self.G.T))
        return v,w
    
    def measurementUpdateGPS(self,y,R):
        if y.shape is (2,):
            y = y.reshape((1,2))
        if y.dtype is not np.double:
            y = y.astype(np.double)
        innovation = y - np.dot(self.C_gps,self.x_hat)
        S = np.dot(self.C_gps,np.dot(self.P,self.C_gps.T)) 
        S += R
        K = np.dot(self.P,np.dot(self.C_gps.conj().T,np.linalg.inv(S)))
        with self.state_lock:
            self.x_hat = self.x_hat + np.dot(K,innovation)
        self.P = np.dot((np.eye(self.__nx) - np.dot(K,self.C_gps)),self.P)
        return innovation, S, K
    
    def measurementUpdateAHRS(self,y):
        y = wrapToPi(y)
        # if y.dtype is not np.double:
        #     y = y.astype(np.double)
        innovation = y - np.dot(self.C_imu, self.x_hat)
        innovation = wrapToPi(innovation)
        S = np.dot(self.C_imu,np.dot(self.P,self.C_imu.T))
        S += self.R_imu[0,0]
        K = np.dot(self.P,self.C_imu.T/S)
        with self.state_lock:
            self.x_hat += K * innovation
            self.x_hat[2] = wrapToPi(self.x_hat[2])
        self.P = np.dot((np.eye(self.__nx) - \
                np.dot(K.reshape((self.__nx,1)),self.C_imu.reshape((1,self.__nx)))),self.P)
        return innovation, S, K
    
    def getYaw(self):
        with self.state_lock:
            return self.x_hat[2]
    
    def getNorthing(self):
        with self.state_lock:
            return self.x_hat[1]
    
    def getEasting(self):
        with self.state_lock:
            return self.x_hat[0]

    def getYawBias(self):
        with self.state_lock:
            return self.x_hat[6]

    def getStateString(self):
        with self.state_lock:
            string = ''
            for ii in range(7):
                string += str(self.x_hat[ii]) + ","
            return string

