import numpy as np

class AutomowEKF:
    __nx = 6                    # Number of States in the Kalman Filter        
    __ny_gps = 2                # Number of measurements from the GPS
    __ny_imu = 1                # Number of measurements from the IMU
    __nu = 2                    # Number of inputs
    __prev_time = 0
    __dt = np.double

    C_gps = np.array([[1,0,0,0,0,0],
                      [0,1,0,0,0,0]],dtype=__dt)
    C_imu = np.array([0,0,1,0,0,0],dtype=__dt)

    def __init__(self,
            x_hat_i, 
            P_i,
            Q,
            R_gps,
            R_imu):
        """Initialize the Kalman Filter with a set of input arguments

        Keyword arguments:
        x_hat_i -- The initial state of the Kalman Estimator
        P_i -- The initial covariance matrix of the Kalman Estimator
        Q -- The process noise covariance of the Kalman Estimator
        R_gps -- The measurement noise covariance of the GPS input
        R_imu -- The measurement noise covariance of the IMU input
        """
        self.x_hat = x_hat_i
        self.P = P_i
        self.Q = Q
        self.R_gps = R_gps
        self.R_imu = R_imu
        
        self.F = np.zeros((self.__nx,self.__nx),dtype=self.__dt)
        self.G = np.zeros((self.__nx,self.__nx),dtype=self.__dt)

    @classmethod
    def fromDefault(cls):
        x_hat_i = np.array([0,0,0,0.159,0.159,0.5461],dtype=cls.__dt)
        P_i = np.diag(np.array([100,100,100,1e-3,1e-3,1e-3],dtype=cls.__dt))
        Q = np.diag(np.array([0.2,0.2,0,0,0,0],dtype=cls.__dt))
        R_gps = np.eye(2,dtype=cls.__dt) * 0.1
        R_imu = np.eye(1,dtype=cls.__dt) * 0.012
        return cls(x_hat_i,P_i,Q,R_gps,R_imu)

    def updateModel(self,u,dt):
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

        return

    def timeUpdate(self,u,time):
        dt = time - self.__prev_time
        self.__prev_time = time
        self.updateModel(u,dt)

        v = self.x_hat[4]/2.0 * u[1] + self.x_hat[3]/2.0 * u[0]
        w = self.x_hat[4]/self.x_hat[5] * u[1] - \
                self.x_hat[3]/self.x_hat[5] * u[0]

        self.x_hat[0] += dt * v * np.cos(self.x_hat[2] + dt * w/2.0)
        self.x_hat[1] += dt * v * np.sin(self.x_hat[2] + dt * w/2.0)
        self.x_hat[2] += dt * w
        self.x_hat[2] = self.wrapToPi(self.x_hat[2])
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
        self.x_hat = self.x_hat + np.dot(K,innovation)
        self.P = np.dot((np.eye(self.__nx) - np.dot(K,self.C_gps)),self.P)
        return innovation, S, K

    def measurementUpdateAHRS(self,y):
        innovation = y - self.C_imu * self.x_hat
        S = np.dot(self.C_imu,np.dot(self.P,self.C_imu.conj().T))
        K = np.dot(self.P,np.dot(self.C_imu.conj().T,np.linalg.inv(S)))
        self.x_hat = self.x_hat + np.dot(K,innovation)
        self.P = np.dot((np.eye(self.__nx) - np.dot(K,self.C_imu)),self.P)
        return

    def getYaw(self):
        return self.x_hat[2]

    def getNorthing(self):
        return self.x_hat[1]

    def getEasting(self):
        return self.x_hat[0]

    @classmethod
    def wrapToPi(self,angle):
        return np.mod(angle+np.pi,2.0*np.pi)-np.pi
