class globals:

    filebase_path = './Path-1'  # 'Path-1' (3d-constant velocity motion) , 'Path-2' (3d-dynamic motion)
    imu_data_flag = 0           # '0' (imu data: x, y, z) , '1' (imu data: ax, ay, az)
    fs_imu = 100.0              # IMU sample frequency
    fs_gps = 10.0               # GPS sample frequency
    R = 1.0                     # Meaurement noise value
    Q = 0.1                     # Process noise value
