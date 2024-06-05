from vehicle import Driver
from controller import GPS, Accelerometer, Gyro
from kalman_filter import LinearKFGPSAccelerometerGyro2D
import pandas as pd
import math
import numpy as np
import random

# Initialize the driver and set initial speed
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
initial_state = [0, 0, 0, 0, 0]  # Initial state (position, velocity, orientation)
initial_covariance = np.eye(5)  # Initial covariance matrix
process_noise = np.eye(5) * 0.01  # Process noise covariance matrix
measurement_noise = np.eye(3) * 0.1   # Measurement noise covariance matrix

# Create the Kalman Filter instance
kf = LinearKFGPSAccelerometerGyro2D(initial_state, initial_covariance, process_noise, measurement_noise)

# Variable to track the last GPS update time
last_gps_time = -1

# Initialize a DataFrame to store the data
data = []

# Main simulation loop
while driver.step() != -1:
    # Get the current simulation time in seconds
    current_time = driver.getTime()
    
    # Change speed
    if int(current_time) % 3 == 0:
        driver.setCruisingSpeed(random.uniform(5, 30))

    # Turn
    if 30 < current_time < 30.2:
        driver.setSteeringAngle(-0.5)  # Turn angle
    else:
        driver.setSteeringAngle(0)

    # Skip Kalman filter calculations for the first 1 second
    if current_time < 1:
        continue
    
    # Read the GPS data
    gps_updated = False
    if int(current_time) != last_gps_time:
        last_gps_time = int(current_time)
        position = gps.getValues()
        gps_x, gps_y, gps_z = position
        gps_updated = True
    
    # Read the accelerometer data
    accel = accelerometer.getValues()
    ax, ay, az = accel
    
    # Read the gyro data
    gyro_data = gyro.getValues()
    gx, gy, gz = gyro_data
    yaw_rate = gz  # Assuming gz is the yaw rate
    
    # Kalman filter prediction step
    kf.predict(dt=accel_sampling_rate / 1000.0, acceleration=[ax, ay], angular_velocity=yaw_rate)
    
    # Kalman filter update step with GPS data
    if gps_updated:
        if not (np.isnan(gps_x) or np.isnan(gps_y)):
            kf.update([gps_x, gps_y])
    
    # Extract the position, velocity, and direction from the state vector
    pos_x, pos_y, vel_x, vel_y, yaw = kf.state.flatten()
    speed = math.sqrt(vel_x**2 + vel_y**2)
    direction = math.degrees(yaw)  # Convert yaw to degrees
    
    # Append the data to the list
    data.append([current_time, gps_x, gps_y, gps_z, ax, ay, az, gx, gy, gz, pos_x, pos_y, vel_x, vel_y, speed, direction])

# Create a DataFrame from the collected data
df = pd.DataFrame(data, columns=['Time', 'GPS_X', 'GPS_Y', 'GPS_Z', 'Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Pos_X', 'Pos_Y', 'Vel_X', 'Vel_Y', 'Speed', 'Direction'])

# Export the DataFrame to a CSV file
df.to_csv('equipment_data.csv', index=False)
