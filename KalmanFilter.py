import numpy as np
from globals import globals

class KalmanFilter:

    def __init__(self, dt, process_noise_cov, measurement_noise_cov, state_cov, initial_state_vector, state_dim=6):
        
        self.dt = dt  # Time step
        self.state_dim = state_dim
        
        # Initialize state vector [x, x_vel, y, y_vel, z, z_vel]
        self.state = np.array(initial_state_vector).reshape((6, 1))

        
        # State transition matrix (F)
        self.F = np.array([[1, dt, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
                           [0, 0, 1, dt, 0, 0],
                           [0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, dt],
                           [0, 0, 0, 0, 0, 1]])
        
        # Process noise covariance (Q) 
        self.Q = process_noise_cov
        
        # Measurement noise covariance (R)
        self.R = measurement_noise_cov
        
        # State covariance matrix (P)
        self.P = state_cov

        # Measurement matrix (H) (We are only measuring position [x, y, z])
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 0, 1, 0]])

    def predict(self, imu_position):

        if(globals.imu_data_flag==1):

            # Prediction step using IMU acceleration data (ax, ay, az)
            ax, ay, az = imu_position
            
            # Update velocity using acceleration
            self.state[1] += ax * self.dt  # Update x velocity
            self.state[3] += ay * self.dt  # Update y velocity
            self.state[5] += az * self.dt  # Update z velocity

            # Update position using the updated velocity
            self.state[0] += self.state[1] * self.dt  # Update x position
            self.state[2] += self.state[3] * self.dt  # Update y position
            self.state[4] += self.state[5] * self.dt  # Update z position

        else:
            # Prediction step using IMU position data (x, y, z)
            imu_x, imu_y, imu_z = imu_position
            
            # Predict state assuming the new position is related to velocity
            # We assume constant velocity model in between IMU position updates
            self.state[0] = imu_x  # Set the predicted x position to the IMU's position
            self.state[2] = imu_y  # Set the predicted y position to the IMU's position
            self.state[4] = imu_z  # Set the predicted z position to the IMU's position

        # Use the state transition matrix to predict the state
        self.state = np.dot(self.F, self.state)

        # Update state covariance matrix
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

        return self.state

    def update(self, gps_position):

        # Measurement update using GPS position data (x, y, z)
        gps_x, gps_y, gps_z = gps_position
        z = np.array([[gps_x], [gps_y], [gps_z]])
        
        # Kalman Gain (K)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        # Update the state with the GPS measurement
        self.state = self.state + np.dot(K, (z - np.dot(self.H, self.state)))
        
        # Update state covariance matrix
        I = np.eye(self.state_dim)
        self.P = np.dot(I - np.dot(K, self.H), self.P)

        return self.state
