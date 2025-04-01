#!/usr/bin/env python

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import time
import math

import numpy as np

try:
    sys.path.append(glob.glob('./PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.tools.scenario_helper import get_waypoint_in_distance, choose_at_junction
from agents.navigation.local_planner import RoadOption
from agents.tools.misc import vector

_HOST_ = '127.0.0.1'
_PORT_ = 2000
_SLEEP_TIME_ = .5



def main():
    vehicles = []
    # try:
    client = carla.Client(_HOST_, _PORT_)
    client.set_timeout(2.0)
    world = client.get_world()

    CarlaDataProvider.set_client(client)
    CarlaDataProvider.set_world(world)
    
    # print(help(t))
    # print("(x,y,z) = ({},{},{})".format(t.location.x, t.location.y,t.location.z))

    # original_settings = world.get_settings()
    # settings = world.get_settings()
    # if not settings.synchronous_mode:
    #     settings.synchronous_mode = True
    #     settings.fixed_delta_seconds = 0.05
    # world.apply_settings(settings)

    # traffic_manager = client.get_trafficmanager()
    # traffic_manager.set_synchronous_mode(True)
    
    carla_map = world.get_map()
    data_provider_map = CarlaDataProvider.get_map()

    # get_waypoint()

    settings = world.get_settings()
    print(settings)
    settings.tile_stream_distance = 1000
    settings.actor_active_distance = 1000
    settings.spectator_as_ego = False
    world.apply_settings(settings)
    settings = world.get_settings()
    print(settings)

    spectator = world.get_spectator()
    location = carla.Location(x=-3897.661133, y=2095.217041, z=344.821991)
    
    
    _second_lane_start = carla.Transform(carla.Location(x=287.214508, y=37.715034, z=2.104192), carla.Rotation(pitch=357.493896, yaw=0.977936, roll=0.000000))
    _second_lane_end = carla.Transform(carla.Location(x=-262.951996, y=407.801849, z=0.000000), carla.Rotation(pitch=360.000000, yaw=179.791809, roll=0.000000))

    _last_lane_start = carla.Transform(carla.Location(x=285.024139, y=30.676624, z=2.206451), carla.Rotation(pitch=357.433716, yaw=0.977936, roll=0.000000))
    _last_lane_end = carla.Transform(carla.Location(x=-225.769119, y=414.666809, z=0.000000), carla.Rotation(pitch=360.000000, yaw=179.791809, roll=0.000000))

    start = carla.Transform(carla.Location(x=-15.514617, y=113.041733, z=0.000000), carla.Rotation(pitch=365.393707, yaw=74.642715, roll=0.000000))


    
    spectator.set_transform(start)

    while(True):
        t = world.get_spectator().get_transform()

        current_location = carla.Location(x=t.location.x, y=t.location.y, z=t.location.z)

        # print(current_location)

        current_waypoint = carla_map.get_waypoint(current_location)

        # mapo = CarlaDataProvider.get_map(current_location)
        #print(mapo)
        if current_waypoint.is_junction:
            for i in current_waypoint.get_junction().get_waypoints(carla.LaneType.Driving):
                print(i[1].road_id)

        # print(current_waypoint.transform.location)
        # print(current_waypoint.road_id)
        time.sleep(_SLEEP_TIME_)

        # break
        print("")




if __name__ == '__main__':
	main()


