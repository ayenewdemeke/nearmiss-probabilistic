from vehicle import Driver
from controller import GPS, Accelerometer, Gyro, Emitter, Receiver
from sfusion.kalman import EKFGPSAccelerometerGyro2D
import pandas as pd
import math
import numpy as np
import random

# Initialize the driver and set initial speed
driver = Driver()
driver.setCruisingSpeed(30)

# Define the sensor sampling rates
gps_sampling_rate = 1000  # 1000 milliseconds for 1 Hz
accel_gyro_sampling_rate = 50  # 50 milliseconds for 20 Hz

# Get the GPS device and enable it with the sampling rate
gps = driver.getDevice('gps')
gps.enable(gps_sampling_rate)

# Get the accelerometer device and enable it with the sampling rate
accelerometer = driver.getDevice('accelerometer')
accelerometer.enable(accel_gyro_sampling_rate)

# Get the gyro device and enable it with the sampling rate
gyro = driver.getDevice('gyro')
gyro.enable(accel_gyro_sampling_rate)

# Initialize the emitter
emitter_equipment_2 = driver.getDevice('emitter_equipment_2')

# Initialize the receiver
receiver_equipment_2 = driver.getDevice('receiver_equipment_2')
receiver_equipment_2.enable(32)

# Kalman filter variables
initial_state = [100, -1.85, 0, 0, 0]  # Initial state (position, velocity, orientation)
initial_covariance = np.eye(5)  # Initial covariance matrix
process_noise = np.eye(5) * 0.01  # Process noise covariance matrix
measurement_noise = np.eye(2) * 0.1  # Measurement noise covariance matrix

# Create the Kalman Filter instance
kf = EKFGPSAccelerometerGyro2D(initial_state, initial_covariance, process_noise, measurement_noise)

# Variable to track the last GPS update time
last_gps_update_time = -1

# Initialize a list to store the data
data = []

# Accelerometer bias
accel_x_bias = -0.90416  # This value is based on your provided data

# Main simulation loop
while driver.step() != -1:
    # Get the current simulation time in seconds
    current_time = driver.getTime()

    # Listen for a stop message from the supervisor and stop
    if receiver_equipment_2.getQueueLength() > 0:
        message = receiver_equipment_2.getString()
        receiver_equipment_2.nextPacket()
        
        if message == "STOP":
            driver.setCruisingSpeed(0)

    # Initialize variables to store sensor data for this iteration
    gps_x, gps_y, gps_z = None, None, None
    
    # Check if it's time to read the GPS data
    if int(current_time) != last_gps_update_time:
        last_gps_update_time = int(current_time)
        
        # Read the GPS data
        position = gps.getValues()
        gps_x, gps_y, gps_z = position
        
        # Kalman filter update step with GPS data
        if not (np.isnan(gps_x) or np.isnan(gps_y)):
            kf.update([gps_x, gps_y])
    
    # Read the accelerometer and gyro data every 50 milliseconds
    if int(current_time * 1000) % accel_gyro_sampling_rate == 0:
        # Read the accelerometer data
        accel = accelerometer.getValues()
        ax, ay, az = accel
        
        # Correct the accelerometer x bias
        ax -= accel_x_bias
        
        # Read the gyro data
        gyro_data = gyro.getValues()
        gx, gy, gz = gyro_data
        yaw_rate = gz  # Assuming gz is the yaw rate
        
        # Kalman filter prediction step
        kf.predict(dt=accel_gyro_sampling_rate / 1000.0, control_input=[ax, ay, yaw_rate])
        
        # Extract the position, velocity, and direction from the state vector
        pos_x, pos_y, vel_x, vel_y, yaw = kf.state.flatten()
        speed = math.sqrt(vel_x**2 + vel_y**2)
        direction = math.degrees(yaw)  # Convert yaw to degrees
        
        # Prepare the data to send
        data_to_send = f"{pos_x},{pos_y},{vel_x},{vel_y},{yaw}"
        emitter_equipment_2.send(data_to_send.encode('utf-8'))
        
        # Append the data to the list
        data.append([current_time, gps_x, gps_y, gps_z, ax, ay, az, gx, gy, gz, pos_x, pos_y, vel_x, vel_y, speed, direction])

# Create a DataFrame from the collected data
df = pd.DataFrame(data, columns=['Time', 'GPS_X', 'GPS_Y', 'GPS_Z', 'Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Pos_X', 'Pos_Y', 'Vel_X', 'Vel_Y', 'Speed', 'Direction'])

# Export the DataFrame to a CSV file
df.to_csv('equipment_data.csv', index=False)
