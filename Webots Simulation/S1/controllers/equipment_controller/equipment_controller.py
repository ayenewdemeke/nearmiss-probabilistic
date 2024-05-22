from vehicle import Driver
from controller import GPS, Accelerometer, Gyro
import pandas as pd
import math
import numpy as np

# Initialize the driver and set cruising speed
driver = Driver()
driver.setCruisingSpeed(10)

# Define the sensor sampling rates
gps_sampling_rate = 1000  # 1000 milliseconds for 1 Hz
accel_sampling_rate = 10  # 10 milliseconds for 100 Hz
gyro_sampling_rate = 10   # 10 milliseconds for 100 Hz

# Get the GPS device and enable it with the sampling rate
gps = driver.getDevice('gps')
gps.enable(gps_sampling_rate)

# Get the accelerometer device and enable it with the sampling rate
accelerometer = driver.getDevice('accelerometer')
accelerometer.enable(accel_sampling_rate)

# Get the gyro device and enable it with the sampling rate
gyro = driver.getDevice('gyro')
gyro.enable(gyro_sampling_rate)

# Kalman filter variables
dt = accel_sampling_rate / 1000.0  # Time step in seconds
A = np.array([[1, 0, dt, 0, 0],
              [0, 1, 0, dt, 0],
              [0, 0, 1, 0, 0],
              [0, 0, 0, 1, 0],
              [0, 0, 0, 0, 1]])  # State transition matrix

H = np.array([[1, 0, 0, 0, 0],
              [0, 1, 0, 0, 0],
              [0, 0, 0, 0, 1]])  # Measurement matrix

Q = np.eye(5) * 0.01  # Process noise covariance matrix
R = np.eye(3) * 0.1   # Measurement noise covariance matrix

x = np.zeros((5, 1))  # Initial state (position, velocity, yaw)
P = np.eye(5)  # Initial covariance matrix

# Initialize a DataFrame to store the data
data = []

# Main simulation loop
while driver.step() != -1:
    # Get the current simulation time in seconds
    current_time = driver.getTime()

    # Turn at 2.5 seconds and revert at 5 seconds
    if 4 < current_time < 4.2:
        driver.setSteeringAngle(-0.5)  # Turn angle
    else:
        driver.setSteeringAngle(0)

    # Skip Kalman filter calculations for the first 1 second
    if current_time < 1:
        continue
        
    # Read the GPS data
    position = gps.getValues()
    gps_x, gps_y, gps_z = position
    
    # Read the accelerometer data
    accel = accelerometer.getValues()
    ax, ay, az = accel
    
    # Read the gyro data
    gyro_data = gyro.getValues()
    gx, gy, gz = gyro_data
    yaw_rate = gz  # Assuming gz is the yaw rate
    
    # Kalman filter prediction step
    F = np.array([[1, 0, dt, 0, 0],
                  [0, 1, 0, dt, 0],
                  [0, 0, 1, 0, 0],
                  [0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 1]])
    B = np.array([[0.5 * dt**2, 0],
                  [0, 0.5 * dt**2],
                  [dt, 0],
                  [0, dt],
                  [0, 0]])
    u = np.array([[ax],
                  [ay]])
    x = F @ x + B @ u
    x[4] += yaw_rate * dt  # Integrate yaw rate to get yaw angle
    P = F @ P @ F.T + Q
    
    # Check if GPS data is valid before updating
    if not (np.isnan(gps_x) or np.isnan(gps_y)):
        # Calculate yaw from velocity
        vel_x, vel_y = x[2, 0], x[3, 0]
        gps_yaw = math.atan2(vel_y, vel_x)
        
        # Kalman filter update step with GPS data and yaw
        Z = np.array([[gps_x],
                      [gps_y],
                      [gps_yaw]])
        y = Z - H @ x
        S = H @ P @ H.T + R
        
        # Check for valid S to avoid division by zero
        if np.linalg.det(S) != 0:
            K = P @ H.T @ np.linalg.inv(S)
            x = x + K @ y
            P = (np.eye(5) - K @ H) @ P
    
    # Extract the position, velocity, and direction from the state vector
    pos_x, pos_y, vel_x, vel_y, yaw = x.flatten()
    speed = math.sqrt(vel_x**2 + vel_y**2)
    direction = math.degrees(yaw)  # Convert yaw to degrees
    
    # Append the data to the list
    data.append([current_time, gps_x, gps_y, gps_z, ax, ay, az, gx, gy, gz, pos_x, pos_y, vel_x, vel_y, speed, direction])

    # Print or log the sensor data for debugging
    print(f"Time: {current_time}, GPS Position: x={gps_x}, y={gps_y}, z={gps_z}")
    print(f"Kalman Filter Position: x={pos_x}, y={pos_y}, Speed: {speed}, Direction: {direction}")
    print(f"Accelerometer: ax={ax}, ay={ay}, az={az}")
    print(f"Gyro: gx={gx}, gy={gy}, gz={gz}")

# Create a DataFrame from the collected data
df = pd.DataFrame(data, columns=['Time', 'GPS_X', 'GPS_Y', 'GPS_Z', 'Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Pos_X', 'Pos_Y', 'Vel_X', 'Vel_Y', 'Speed', 'Direction'])

# Export the DataFrame to a CSV file
df.to_csv('equipment_data.csv', index=False)
