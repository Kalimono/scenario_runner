import carla

client = carla.Client("localhost", 2000)
client.set_timeout(30.0)

with open("boras.xodr", "r", encoding="utf-8") as f:
    xodr_data = f.read()

params = carla.OpendriveGenerationParameters(
    vertex_distance=2.0,
    max_road_length=50.0,
    wall_height=0.0,
    additional_width=0.6,
    smooth_junctions=True,
    enable_mesh_visibility=True
)

world = client.generate_opendrive_world(xodr_data, params)
print("OpenDRIVE world generated.")

