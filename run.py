import numpy as np
from KalmanFilter import KalmanFilter
import matplotlib.pyplot as plt
import pandas as pd
from globals import globals

# Initialize storage for plotting
kf_values_list = []
gps_update_interval = globals.fs_imu // globals.fs_gps  #GPS update time


# Function to simulate IMU data
def get_imu_data():

    if(globals.imu_data_flag==1):
        # Simulated IMU acceleration data (ax, ay, az) in m/s^2
        df = pd.read_csv(globals.filebase_path+'/accel.csv')
        ax = df['accel_x'].tolist()  # Accelerometer x-axis
        ay = df['accel_y'].tolist()  # Accelerometer y-axis
        az = df['accel_z'].tolist()  # Accelerometer z-axis
        return [ax, ay, az] 

    else:
        # Simulated IMU position data in ECEF (x, y, z)
        df = pd.read_csv(globals.filebase_path+'/imu_pos.csv')
        x = df['pos_x'].tolist()
        y = df['pos_y'].tolist()
        z = df['pos_z'].tolist()
        return [x,y,z]  


# Function to simulate GPS data
def get_gps_data():
    # Simulated GPS position data in ECEF (x, y, z)
    df = pd.read_csv(globals.filebase_path+'/gps.csv')
    x = df['gps_x'].tolist()
    y = df['gps_y'].tolist()
    z = df['gps_z'].tolist()
    return [x,y,z]  

# Function to simulate true position data
def get_true_position():
    # Simulate true position data (x, y, z)
    df = pd.read_csv(globals.filebase_path+'/true_pos.csv')
    x = df['ref_pos_x'].tolist()
    y = df['ref_pos_y'].tolist()
    z = df['ref_pos_z'].tolist()
    
    return [x,y,z]  


# Example usage
def main():

    true_pos = get_true_position()
    imu_data = get_imu_data()
    gps_data = get_gps_data()

    df = pd.read_csv(globals.filebase_path+'/time.csv')
    time_array = df['time']

    # Time step for IMU
    dt = 1/globals.fs_imu

    # Process noise covariance
    process_noise_cov = np.eye(6) * globals.Q

    # Measurement noise covariance
    measurement_noise_cov = np.eye(3) * globals.R

    # Initial state covariance (P)
    state_cov = np.eye(6) * 0.1

    # Initial state vector (x)
    initial_state_vector = [true_pos[0][0], 0, true_pos[1][0], 0, true_pos[2][0], 0]

    # Initialize the Kalman Filter
    kf = KalmanFilter(dt, process_noise_cov, measurement_noise_cov, state_cov, initial_state_vector)

    for t in range(len(time_array)):

        # Perform prediction every time step (100 Hz for IMU)
        predicted_state = kf.predict([imu_data[0][t],imu_data[1][t],imu_data[2][t]])

        # Perform update with GPS data every 1 second
        if t % gps_update_interval == 0:  # GPS at 1Hz
            arr_ind = int(t/gps_update_interval)
            updated_state = kf.update([gps_data[0][arr_ind],gps_data[1][arr_ind],gps_data[2][arr_ind]])
            kf_values_list.append(updated_state[[0, 2, 4]].flatten())

        else:
            kf_values_list.append(predicted_state[[0, 2, 4]].flatten())

    
    # Convert lists to numpy arrays for easier indexing and plotting
    true_values = np.array(true_pos)
    kf_values = np.array(kf_values_list)
    time_steps = np.array(time_array)

    # Plot results for x, y, and z positions
    plt.figure(figsize=(12, 8))

    for i, axis_label in enumerate(['x', 'y', 'z']):
        plt.subplot(3, 1, i + 1)
        plt.plot(time_steps, true_values[i, :], label="True Position", color='blue')
        plt.plot(time_steps, kf_values[:, i], label="Kalman Filter Output", color='red', linestyle=':')
        plt.ylabel(f'{axis_label}-Position (m)')
        plt.legend()
        plt.grid(True)

    plt.xlabel('Time (seconds)')
    plt.suptitle('LCKF IMU and GPS Fusion  - 2D Position Plot')
    plt.tight_layout()

    plt.figure()
    ax = plt.axes(projection ='3d')
    ax.plot3D(true_values[0, :], true_values[1, :], true_values[2, :], label="True Position", color='blue')
    ax.plot3D(kf_values[:, 0], kf_values[:, 1], kf_values[:, 2], label="Kalman Filter Output", color='red', linestyle=':')
    ax.set_title('LCKF IMU and GPS Fusion  - 3D Position Plot')
    ax.set_xlabel('X-Position (m)')
    ax.set_ylabel('Y-Position (m)')
    ax.set_zlabel('Z-Position (m)')
    ax.legend()

    plt.show()



if __name__ == "__main__":
    main()
