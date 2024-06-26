import math
from controller import Supervisor, Node, Receiver, Emitter
import numpy as np
import pandas as pd

def generate_gaussian_ellipse_points(sigma_x, sigma_y, confidence_level=0.3, num_points=100):
    k = math.sqrt(-2 * math.log(1 - confidence_level))
    a = sigma_x * k
    b = sigma_y * k
    points = [(a * math.cos(2 * math.pi * i / num_points), b * math.sin(2 * math.pi * i / num_points), 0) for i in range(num_points)]
    return points, a, b

def create_shaded_ellipse(supervisor, points, translation, rotation, color):
    root = supervisor.getRoot()
    children_field = root.getField("children")

    points_str = ", ".join(f"{point[0]} {point[1]} {point[2]}" for point in points)
    indices = ", ".join(f"{i+1}, {(i+1)%len(points)+1}, 0, -1" for i in range(len(points)))

    ellipse_string = f"""
    Shape {{
        appearance Appearance {{
            material Material {{
                diffuseColor {color[0]} {color[1]} {color[2]}
            }}
        }}
        geometry IndexedFaceSet {{
            coord Coordinate {{
                point [
                    0 0 0, {points_str}
                ]
            }}
            coordIndex [
                {indices}
            ]
        }}
    }}
    """
    transform_string = f"""
    Transform {{
        translation {translation[0]} {translation[1]} {translation[2]}
        rotation {rotation[0]} {rotation[1]} {rotation[2]} {rotation[3]}
        children [
            {ellipse_string}
        ]
    }}
    """
    children_field.importMFNodeFromString(-1, transform_string)

def get_center_position(node, offset=1):
    orientation = node.getOrientation()
    position = node.getPosition()
    
    # Extract the forward vector from the orientation matrix
    forward_vector = [orientation[0], orientation[3], orientation[6]]
    
    # Calculate the new position with the offset
    new_position = [
        position[0] + forward_vector[0] * offset,
        position[1] + forward_vector[1] * offset,
        position[2] + forward_vector[2] * offset
    ]
    
    return new_position

def is_point_in_ellipse(point, center, sigma_x, sigma_y, confidence_level=0.3):
    k = math.sqrt(-2 * math.log(1 - confidence_level))
    a = sigma_x * k
    b = sigma_y * k
    
    x, y, z = point
    cx, cy, cz = center
    
    dx = x - cx
    dy = y - cy
    
    # Check if the point is within the ellipse
    if (dx**2 / a**2 + dy**2 / b**2) <= 1:
        return True
    return False

# Function to check if a point is within a rectangle
def is_point_in_rectangle(point, center, length, width):
    x, y, _ = point
    cx, cy, _ = center
    
    half_length = length / 2
    half_width = width / 2
    
    return (cx - half_length <= x <= cx + half_length) and (cy - half_width <= y <= cy + half_width)
    
def set_worker_position(worker):
    # Define the mean and standard deviation for x and y coordinates
    mean_x, std_x = 280, 5
    mean_y, std_y = -1.85, 3
    
    # Generate x and y values from a normal distribution and limit them within the range
    x = np.random.normal(mean_x, std_x)
    y = np.random.normal(mean_y, std_y)
    
    # Ensure x and y are within the specified range
    x = max(260, min(x, 300))
    y = max(-9.25, min(y, 5.55))
    
    # Set the position of the worker
    worker.getField('translation').setSFVec3f([x, y, 1.27])

def run_hazard_zone_simulation(supervisor, reaction_time=2.5):
    # Reset flags and variables
    alert_triggered_time = None
    brakes_applied = False
    condition_checked = False
    was_in_inner_boundary = False
    was_in_outer_boundary = False
    min_distance = 100

    # Get the equipment node
    equipment = supervisor.getFromDef('EQUIPMENT')
    initial_position = [100, -1.85, 0.6]  # Define initial position

    # Get the worker
    worker = supervisor.getFromDef('WORKER')
    
    # Initialize the receiver and emitter
    receiver_supervisor = supervisor.getDevice('receiver_supervisor')
    receiver_supervisor.enable(32)
    emitter_supervisor = supervisor.getDevice('emitter_supervisor')
    
    translation = None
    
    # Set equipment dimensions in meter and initial speed in km/h
    equipment_length = 3
    equipment_width = 1.7
    equipment_speed = 20

    # Simulation loop
    while supervisor.step(32) != -1:
        # Check for received messages
        if receiver_supervisor.getQueueLength() > 0:
            message = receiver_supervisor.getString()
            receiver_supervisor.nextPacket()
            
            # Handle speed reset messages
            if message.startswith("SPEED"):
                _, speed = message.split(',')
                equipment_speed = float(speed)
                continue  # Continue to the next iteration
            
            # Parse the received message
            pos_x, pos_y, vel_x, vel_y, yaw = map(float, message.split(','))
            velocity = [vel_x, vel_y, 0]  # Assuming 2D velocity

            sigma_x = 2.367 + 0.592 * equipment_length + abs(2.959 * vel_x) + 0.563 * vel_x * vel_x
            sigma_y = 2.367 + 0.592 * equipment_width

            # Remove previous ellipses
            root = supervisor.getRoot()
            children_field = root.getField("children")
            for i in range(children_field.getCount() - 1, -1, -1):
                child = children_field.getMFNode(i)
                if child.getType() == Node.TRANSFORM:
                    children_field.removeMF(i)

            # Generate ellipse points
            points, a, b = generate_gaussian_ellipse_points(sigma_x, sigma_y)

            # Use the received yaw for rotation
            rotation = [0, 0, 1, yaw]  # Rotate around the Z-axis

            # Get the translation of the equipment with an offset to the front
            translation = get_center_position(equipment)
            
            # Draw the shaded ellipse at the adjusted position
            # create_shaded_ellipse(supervisor, points, translation, rotation, (1, 0, 0))

            # Check if the object is within the ellipse
            worker_position = worker.getPosition()
            danger_zone = False

            # Check if the point is within the ellipse
            if is_point_in_ellipse(worker_position, translation, sigma_x, sigma_y):
                danger_zone = True
                if alert_triggered_time is None:
                    alert_triggered_time = supervisor.getTime()  # Record the time the alert is triggered

            # Check if the worker is in the hazard zone and alert
            # if danger_zone:
                # print("Alert! You are in the danger zone!")
            
            # Track worker's position relative to ellipses during the movement
            worker_position = worker.getPosition()
            if is_point_in_rectangle(worker_position, translation, 5, 3.7):
                was_in_inner_boundary = True
            elif is_point_in_rectangle(worker_position, translation, 13, 9.1):
                was_in_outer_boundary = True
            current_distance = math.sqrt((translation[0] - worker_position[0])**2 + (translation[1] - worker_position[1])**2)
            if current_distance < min_distance:
                min_distance = current_distance
                
            # Check if it's time to apply the brakes
            if alert_triggered_time is not None and supervisor.getTime() - alert_triggered_time >= reaction_time:
                emitter_supervisor.send("STOP".encode('utf-8'))
                brakes_applied = True  # Flag to indicate brakes have been applied
            
            # Check if the equipment has come to rest and condition hasn't been checked
            if alert_triggered_time and brakes_applied and not condition_checked and abs(vel_x) < 0.1 and abs(vel_y) < 0.1:
                if was_in_inner_boundary:
                    result = "False Negative"
                elif was_in_outer_boundary:
                    result = "True Positive"
                else:
                    result = "False Positive"
                
                # Set the flag to true to prevent re-checking
                condition_checked = True
                emitter_supervisor.send("RESET".encode('utf-8'))
                set_worker_position(worker) # Set worker to new position
                equipment.getField('translation').setSFVec3f(initial_position)  # Reset equipment position
            
                return equipment_speed, worker_position, min_distance, result  # Return the result for this run
            
            # Check for True Negative case if condition has not been checked yet
            else:
                worker_position = worker.getPosition()
                
                # Apply brakes if the equipment has passed the worker
                if translation[0] > worker_position[0] and not brakes_applied:
                    emitter_supervisor.send("STOP".encode('utf-8'))
                    brakes_applied = True
                
                # Ensure the equipment has come to a stop before classifying as True Negative
                elif brakes_applied and abs(vel_x) < 0.1 and abs(vel_y) < 0.1:
                    result = "True Negative"
                    condition_checked = True
                    emitter_supervisor.send("RESET".encode('utf-8'))
                    set_worker_position(worker) # Set worker to new position
                    equipment.getField('translation').setSFVec3f(initial_position)  # Reset equipment position
                    
                    return equipment_speed, worker_position, min_distance, result  # Return the result for this run    

# Main script
supervisor = Supervisor()
SIMULATION_RUN_COUNT = 100  # Number of simulation runs

results = []

for i in range(SIMULATION_RUN_COUNT):
    print(f"Running simulation {i + 1}")
    equipment_speed, worker_position, distance, result = run_hazard_zone_simulation(supervisor, reaction_time=2.5)
    results.append((i + 1, equipment_speed, worker_position, distance, result))
    print(f"Iteration {i + 1}: {result}")

# Save results to a CSV file
df = pd.DataFrame(results, columns=["Iteration", "Vehicle Speed", "Worker Location", "Final Spacing", "Result"])
df["Vehicle Speed"] = df["Vehicle Speed"].round(2)
df["Final Spacing"] = df["Final Spacing"].round(2)
df["Worker Location"] = df["Worker Location"].apply(lambda x: f"{round(x[0], 2)}, {round(x[1], 2)}, {round(x[2], 2)}")
df.to_csv("hazard_zone_simulation_results.csv", index=False)
print("Simulation results saved to hazard_zone_simulation_results.csv")