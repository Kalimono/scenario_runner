import carla

# Read OSM file
with open("boras.osm", "r", encoding="utf-8") as f:
    osm_data = f.read()

# Configure conversion
settings = carla.Osm2OdrSettings()

settings.default_lane_width = 3.5
settings.elevation_layer_height = 4.0  # meters per OSM layer level; bridges (layer=1) → 4m above ground
settings.generate_traffic_lights = True
settings.all_junctions_with_traffic_lights = False

# Convert
xodr_data = carla.Osm2Odr.convert(osm_data, settings)

# Save result
with open("boras.xodr", "w", encoding="utf-8") as f:
    f.write(xodr_data)

print("OpenDRIVE file generated.")

Hello Claude, I need your help to create a scenario. You may use the general structure of the FollowLeadingVehicle scenario.

The ego vehicle should spawn at carla.Transform(carla.Location(x=291.08, y=-213.89, z=0.20), carla.Rotation(pitch=-20.24, yaw=179.41, roll=-0.00)) and as soon as it starts moving a placeholder behavior named "play_mp3" should be activated. This behavior should be defined in a new file at scenariomanager/scenarioatomics/atomic_behaviors_custom.py. This behavior should be used throughout the scenario to guide the driver. When setting up the py_tree, the play_mp3 behaviors should be in its own branch from root in order to now mess other things up.

[carla.Transform(carla.Location(x=296.79, y=-168.97, z=0.20), carla.Rotation(pitch=0.00, yaw=0.33, roll=0.00)) "RIGHT"]
[carla.Transform(carla.Location(x=310.74, y=-141.19, z=0.00), carla.Rotation(pitch=0.00, yaw=90.51, roll=0.00)) "STRAIGHT"]
[carla.Transform(carla.Location(x=342.56, y=-64.54, z=0.00), carla.Rotation(pitch=0.00, yaw=-2.29, roll=0.00)),
"RIGHT ONTO HIGHWAY"]
[carla.Transform(carla.Location(x=299.49, y=13.42, z=1.62), carla.Rotation(pitch=2.20, yaw=-179.02, roll=0.00)),
"ACCIDENT AHEAD"] 

Vehicle 1: carla.Transform(carla.Location(x=-390.85, y=6.40, z=1.00), carla.Rotation(pitch=-69.25, yaw=119.04, roll=-0.01)),
Vehicle 2: carla.Transform(carla.Location(x=-384.93, y=6.43, z=-0.00), carla.Rotation(pitch=-71.81, yaw=178.91, roll=-0.01)),
Ambulance 1: carla.Transform(carla.Location(x=-379.08, y=10.53, z=-0.01), carla.Rotation(pitch=-74.89, yaw=-179.86, roll=-0.01)),
Ambulance 2: carla.Transform(carla.Location(x=-405.22, y=7.52, z=0.00), carla.Rotation(pitch=-86.38, yaw=169.75, roll=-0.01)),

[carla.Transform(carla.Location(x=7.71, y=-184.95, z=0.00), carla.Rotation(pitch=0.00, yaw=-90.22, roll=0.00)), "RIGHT OFF HIGHWAY"]
[carla.Transform(carla.Location(x=180.30, y=-364.47, z=0.00), carla.Rotation(pitch=0.00, yaw=0.50, roll=0.00)), "ENTERING RESIDENTIAL AREA"]
[carla.Transform(carla.Location(x=202.50, y=-340.14, z=0.02), carla.Rotation(pitch=0.00, yaw=91.31, roll=0.00)), "STRAIGHT"]
[carla.Transform(carla.Location(x=201.34, y=-289.70, z=0.02), carla.Rotation(pitch=0.00, yaw=91.31, roll=0.00)), "LEFT"]
[carla.Transform(carla.Location(x=224.47, y=-246.00, z=0.00), carla.Rotation(pitch=360.00, yaw=359.61, roll=0.00)), "STRAIGHT"]