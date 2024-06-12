import math
from controller import Supervisor, Node, Receiver, Emitter

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

def get_front_position(node, offset=1.0):
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

def is_point_in_ellipse(point, center, sigma_x, sigma_y, confidence_level=0.95):
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

def is_within_radius(point1, point2, radius):
    dx = point1[0] - point2[0]
    dy = point1[1] - point2[1]
    dz = point1[2] - point2[2]
    distance = math.sqrt(dx**2 + dy**2 + dz**2)
    return distance <= radius

supervisor = Supervisor()

# Get references to the equipment and the static zone
equipment_1 = supervisor.getFromDef("EQUIPMENT_1")
static_warning_zone = supervisor.getFromDef("STATIC_WARNING_ZONE")

# Get the equipment node
equipment_2 = supervisor.getFromDef('EQUIPMENT_2')

# Get the worker
object_node = supervisor.getFromDef('WORKER')

# Initialize the receiver and emitter
receiver_supervisor = supervisor.getDevice('receiver_supervisor')
receiver_supervisor.enable(32)
emitter_supervisor = supervisor.getDevice('emitter_supervisor')

kx = 2
ky = 10

# Variable to track the last position for direct velocity calculation
last_position = None
last_vel_x = 0
last_vel_y = 0

# Timer for handling the reaction delay
alert_triggered_time = None
reaction_time = 2.5  # Time in seconds to apply brakes

# Flag to ensure brakes are applied only once
brakes_applied = False

# Variable to track the final stopping position of the equipment
final_position = None

# Flag to ensure the condition check is done only once
condition_checked = False

# Simulation main loop
while supervisor.step(32) != -1:
    
    # Update the position of the static zone to match the equipment's position
    equipment_position = equipment_1.getField("translation").getSFVec3f()
    static_warning_zone.getField("translation").setSFVec3f(equipment_position)
    
    # Check for received messages
    if receiver_supervisor.getQueueLength() > 0:
        message = receiver_supervisor.getString()
        receiver_supervisor.nextPacket()
        
        # Parse the received message
        pos_x, pos_y, vel_x, vel_y, yaw = map(float, message.split(','))
        velocity = [vel_x, vel_y, 0]  # Assuming 2D velocity

        sigma_x = 5.325 + abs(2.959 * vel_x) + 0.422 * vel_x * vel_x
        sigma_y = 3.846

        confidence_intervals = [0.3]  # Only the danger zone
        colors = [(1, 0, 0)]  # Red
        z_offsets = [0.2]  # Z offsets for visibility

        # Remove previous ellipses
        root = supervisor.getRoot()
        children_field = root.getField("children")
        for i in range(children_field.getCount() - 1, -1, -1):
            child = children_field.getMFNode(i)
            if child.getType() == Node.TRANSFORM:
                children_field.removeMF(i)

        # Draw ellipses for different confidence levels
        for confidence_level, color, z_offset in zip(confidence_intervals, colors, z_offsets):
            points, a, b = generate_gaussian_ellipse_points(sigma_x, sigma_y, confidence_level)

            # Calculate the rotation for the ellipse based on the computed velocities
            if vel_x != 0 or vel_y != 0:
                direction_rad = math.atan2(vel_y, vel_x)
            else:
                direction_rad = 0
            rotation = [0, 0, 1, direction_rad]  # Rotate around the Z-axis

            # Get the translation of the equipment with an offset to the front
            translation = [pos_x, pos_y, 0]  # Use the received position
            translation = get_front_position(equipment_2)
            translation[2] += z_offset  # Add the z offset

            # Draw the shaded ellipse at the adjusted position
            create_shaded_ellipse(supervisor, points, translation, rotation, color)

        # Check if the object is within the ellipses
        object_position = object_node.getPosition()
        danger_zone = False

        if is_point_in_ellipse(object_position, translation, sigma_x, sigma_y, 0.3):
            danger_zone = True
            if alert_triggered_time is None:
                alert_triggered_time = supervisor.getTime()  # Record the time the alert is triggered

        # Check if the worker is in the hazard zone and alert
        if is_within_radius(object_position, equipment_position, 10) or danger_zone:
            print("Alert! You are in the danger zone!")
            
        # Check if it's time to apply the brakes
        if alert_triggered_time is not None and supervisor.getTime() - alert_triggered_time >= reaction_time:
            emitter_supervisor.send("STOP".encode('utf-8'))
            brakes_applied = True  # Flag to indicate brakes have been applied
        
        # Check if the equipment has come to rest
        if brakes_applied and abs(vel_x) < 0.1 and abs(vel_y) < 0.1 and not condition_checked:
            final_position = equipment_2.getField("translation").getSFVec3f()  # Record the final stopping position
            worker_position = object_node.getPosition()
            distance = math.sqrt((final_position[0] - worker_position[0])**2 + (final_position[1] - worker_position[1])**2)
        
            if distance < 1:
                print(f"False Negative: The equipment hits the worker or comes within 1 meter before stopping. Actual distance: {distance:.2f} meters.")
            elif 1 <= distance <= 4.5:
                print(f"True Positive: The equipment stops with the worker 1 to 4.5 meters away. Actual distance: {distance:.2f} meters.")
            elif distance > 4.5:
                print(f"False Positive: The equipment stops with the worker more than 4.5 meters away. Actual distance: {distance:.2f} meters.")
            else:
                print(f"True Negative: No worker enters the hazard zone and no worker is within 4.5 meters. Actual distance: {distance:.2f} meters.")
        
            # Set the flag to true to prevent re-checking
            condition_checked = True

