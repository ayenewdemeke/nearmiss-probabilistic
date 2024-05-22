from controller import Supervisor

# Create the Supervisor instance
supervisor = Supervisor()

# Get the root node of the scene
root = supervisor.getRoot()
root_children_field = root.getField("children")

# Define the circular danger zone as a string
danger_zone_def = """
Solid {
  translation 0 0.01 0
  children [
    Shape {
      appearance Appearance {
        material Material {
          diffuseColor 1 0 0
          transparency 0.5
        }
      }
      geometry Cylinder {
        radius 1.0  # Adjust the radius as needed
        height 0.01  # Small height to make it a flat circle
      }
    }
  ]
}
"""

# Add the circular danger zone to the scene
root_children_field.importMFNodeFromString(-1, danger_zone_def)

# Main simulation loop
while supervisor.step(int(supervisor.getBasicTimeStep())) != -1:
    # Update the position of the danger zone based on the equipment's position if needed
    equipment_node = supervisor.getFromDef("EQUIPMENT_DEF")  # Replace with the DEF name of your equipment
    if equipment_node:
        equipment_position = equipment_node.getField("translation").getSFVec3f()
        danger_zone_node = supervisor.getFromDef("DANGER_ZONE_DEF")  # Replace with the DEF name of your danger zone
        if danger_zone_node:
            danger_zone_node.getField("translation").setSFVec3f(equipment_position)
