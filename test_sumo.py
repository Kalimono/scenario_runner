import carla

client = carla.Client("localhost", 2000)
client.set_timeout(20)

with open("boras.xodr") as f:
    xodr = f.read()

params = carla.OpendriveGenerationParameters()
params.vertex_distance = 2.0
params.smooth_junctions = True

world = client.generate_opendrive_world(xodr, params)