import numpy as np

class AutomowEKF:
    __nx = 6                    # Number of States in the Kalman Filter        
    __ny_gps = 2                # Number of measurements from the GPS
    __ny_imu = 1                # Number of measurements from the IMU
    __nu = 2                    # Number of inputs
    __prev_time = 0

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
        
        self.F = np.zeros((self.__nx,self.__nx))
        self.G = np.zeros((self.__nx,self.__nx))

    @classmethod
    def fromDefault(cls):
        x_hat_i = np.array([0,0,0,1,1,1])
        P_i = np.diag(np.array([1,1,1,1,1,1]))
        Q = np.diag(np.array([1,1,0,0,0,0]))
        R_gps = np.eye(2)
        R_imu = np.eye(1)
        return cls(x_hat_i,P_i,Q,R_gps,R_imu)

    def UpdateModel(self,u,dt):
        self.F = np.eye(self.__nx)
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
        
        self.G = np.zeros((self.__nx,self.__nx))
        self.G[0,0] = 0.5 * dt * self.x_hat[3] * np.cos(self.x_hat[2])
        self.G[0,1] = 0.5 * dt * self.x_hat[4] * np.cos(self.x_hat[2])
        self.G[0,3] = 0.5 * dt * u[0] * np.cos(self.x_hat[2])
        self.G[0,4] = 0.5 * dt * u[1] * np.cos(self.x_hat[2])
        self.G[1,0] = 0.5 * dt * self.x_hat[3] * np.sin(self.x_hat[2])
        self.G[1,1] = 0.5 * dt * self.x_hat[3] * np.sin(self.x_hat[2])
        self.G[1,3] = 0.5 * dt * u[0] * np.cos(self.x_hat[2])
        self.G[1,4] = 0.5 * dt * u[0] * np.cos(self.x_hat[2])
        self.G[2,0] = -1.0 * dt * self.x_hat[3]/self.x_hat[5]
        self.G[2,1] = dt * self.x_hat[4]/self.x_hat[5]
        self.G[2,2] = dt 
        self.G[2,3] = -1.0 * dt * self.x_hat[3]/self.x_hat[5] 
        self.G[2,4] = dt * self.x_hat[4]/self.x_hat[5] 
        self.G[3,3] = dt 
        self.G[4,4] = dt
        self.G[5,5] = dt

        return

    def TimeUpdate(self,u,time):
        dt = time - self.__prev_time
        self.__prev_time = time

        self.UpdateModel(u,dt)

        v = self.x_hat[4]/2 * u[1] + self.x_hat[5]/2 * u[0]
        w = self.x_hat[4]/self.x_hat[6] * u[1] - \
                self.x_hat[5]/self.x_hat[6] * u[0]

        self.x_hat[0] += dt * v * np.cos(self.x_hat[2] + dt * w/2)
        self.x_hat[1] += dt * v * np.sin(self.x_hat[2] + dt * w/2)
        self.x_hat[2] += dt * w
        self.x_hat[2] = self.wrapToPi(self.x_hat[2])

        self.P = self.F * self.P * self.F.conj().transpose() \
                + self.G * self.Q * self.G.conj().transpose()

        return (self.x_hat, self.P)

    def MeasurementUpdateGPS(self,y,R):
        return (self.x_hat, self.P)

    def MeasurementUpdateIMU(self,y):
        return (self.x_hat, self.P)

    @classmethod
    def wrapToPi(angle):
        angle += np.pi
        is_neg = (angle < 0)
        angle = np.fmod(angle,2*np.pi)
        if is_neg:
            angle += 2*np.pi
        angle -= np.pi
        return angle
