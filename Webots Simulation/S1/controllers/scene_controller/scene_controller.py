import math
from controller import Supervisor, Node

def generate_gaussian_ellipse_points(sigma_x, sigma_y, confidence_level=0.3, num_points=100):
    k = math.sqrt(-2 * math.log(1 - confidence_level))
    a = sigma_x * k
    b = sigma_y * k
    points = [(a * math.cos(2 * math.pi * i / num_points), b * math.sin(2 * math.pi * i / num_points), 0) for i in range(num_points)]
    return points

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

kx = 2
ky = 10

# Variable to track the last position for direct velocity calculation
last_position = None
last_vel_x = 0
last_vel_y = 0

# Simulation main loop
while supervisor.step(32) != -1:
    
    # Update the position of the static zone to match the equipment's position
    equipment_position = equipment_1.getField("translation").getSFVec3f()
    static_warning_zone.getField("translation").setSFVec3f(equipment_position)
    
    # Compute velocities directly from the equipment's own translation coordinates
    current_translation = equipment_2.getPosition()
    if last_position is not None:
        delta_x = current_translation[0] - last_position[0]
        delta_y = current_translation[1] - last_position[1]
        delta_time = supervisor.getTime() - last_position[2]

        if delta_time > 0:
            direct_vel_x = delta_x / delta_time
            direct_vel_y = delta_y / delta_time
        else:
            direct_vel_x, direct_vel_y = 0, 0

        direct_speed = math.sqrt(direct_vel_x**2 + direct_vel_y**2)

        # Calculate the change in the ratio of velocities
        velocity_change_factor = abs((direct_vel_y / (direct_vel_x+1)) - (last_vel_y / (last_vel_x+1)))

    else:
        direct_vel_x, direct_vel_y, direct_speed = 0, 0, 0
        velocity_change_factor = 0

    last_position = (current_translation[0], current_translation[1], supervisor.getTime())
    last_vel_x = direct_vel_x
    last_vel_y = direct_vel_y

    sigma_x = 2.5 + abs(kx * direct_vel_x)
    sigma_y = 2.5 + abs(ky * velocity_change_factor)

    confidence_intervals = [0.75, 0.5, 0.3]
    colors = [(1, 1, 0.2), (1, 0.4, 0), (1, 0, 0)]  # Yellow, Orange, Red
    z_offsets = [0, 0.1, 0.2]  # Z offsets for visibility

    # Remove previous ellipses
    root = supervisor.getRoot()
    children_field = root.getField("children")
    for i in range(children_field.getCount() - 1, -1, -1):
        child = children_field.getMFNode(i)
        if child.getType() == Node.TRANSFORM:
            children_field.removeMF(i)

    # Draw ellipses for different confidence levels
    for confidence_level, color, z_offset in zip(confidence_intervals, colors, z_offsets):
        points = generate_gaussian_ellipse_points(sigma_x, sigma_y, confidence_level)

        # Calculate the rotation for the ellipse based on the computed velocities
        if direct_vel_x != 0 or direct_vel_y != 0:
            direction_rad = math.atan2(direct_vel_y, direct_vel_x)
        else:
            direction_rad = 0
        rotation = [0, 0, 1, direction_rad]  # Rotate around the Z-axis

        # Get the translation of the equipment with an offset to the front
        translation = get_front_position(equipment_2)
        translation[2] += z_offset  # Add the z offset

        # Draw the shaded ellipse at the adjusted position
        create_shaded_ellipse(supervisor, points, translation, rotation, color)

    # Check if the object is within the ellipses
    object_position = object_node.getPosition()
    danger_zone = False
    warning_zone = False
    caution_zone = False

    if is_point_in_ellipse(object_position, translation, sigma_x, sigma_y, 0.75):
        caution_zone = True
    if is_point_in_ellipse(object_position, translation, sigma_x, sigma_y, 0.5):
        warning_zone = True
    if is_point_in_ellipse(object_position, translation, sigma_x, sigma_y, 0.3):
        danger_zone = True

    # Check if the worker is in the hazard zone and alert
    if is_within_radius(object_position, equipment_position, 10):
        print("Alert! You are in the danger zone!")
    elif danger_zone:
        print("Alert! You are in the danger zone!")
    elif warning_zone:
        print("Alert! You are in the warning zone!")
    elif caution_zone:
        print("Alert! You are in the caution zone!")
