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
_SLEEP_TIME_ = .1


def main():
    client = carla.Client(_HOST_, _PORT_)
    client.set_timeout(2.0)
    world = client.get_world()

    CarlaDataProvider.set_client(client)
    CarlaDataProvider.set_world(world)
	
	# print(help(t))
	# print("(x,y,z) = ({},{},{})".format(t.location.x, t.location.y,t.location.z))
	
    carla_map = world.get_map()
    data_provider_map = CarlaDataProvider.get_map()

    # get_waypoint()

    spectator = world.get_spectator()
    location = carla.Location(x=-3925.3, y= 2134.1, z=349.1)
    spectator.set_transform(carla.Transform(location))
	
    # current_distance_from_start = 0

    # current_location = location

    # distance_to_next = 20

    # turn = 0

    # waypoint = data_provider_map.get_waypoint(current_location)

    # while True:
         
        # current_waypoint = data_provider_map.get_waypoint(current_location)
        # # new_waypoint, _ = get_waypoint_in_distance(current_waypoint, current_distance_from_start, stop_at_junction=False)

        # new_waypoint = current_waypoint.next(20)

        # print(new_waypoint[-1])

        # # for i in new_waypoint:
        # #      print(i)

        # spectator.set_transform(new_waypoint[-1].transform)
        # current_distance_from_start += 20

        # time.sleep(_SLEEP_TIME_)

        # reached_junction = False
        # threshold = math.radians(0.1)
        # plan = []

        # wp_choice = waypoint.next(distance_to_next)
        # if len(wp_choice) > 1:
        #     print("crossing")
        #     reached_junction = True
        #     waypoint = choose_at_junction(waypoint, wp_choice, turn)
        # else:
        #     waypoint = wp_choice[0]
        # plan.append((waypoint, RoadOption.LANEFOLLOW))
        # #   End condition for the behavior  
        # if turn != 0 and reached_junction and len(plan) >= 3:

        #     v_1 = vector(
        #         plan[-2][0].transform.location,
        #         plan[-1][0].transform.location)
        #     v_2 = vector(
        #         plan[-3][0].transform.location,
        #         plan[-2][0].transform.location)
        #     angle_wp = math.acos(
        #         np.dot(v_1, v_2) / abs((np.linalg.norm(v_1) * np.linalg.norm(v_2))))
        #     if angle_wp < threshold:
        #         break
        # elif reached_junction and not plan[-1][0].is_intersection:
        #     break

        # # plan[0][0].transform

        # plan[0][0].transform.location.z += 10

        # current_transform = plan[0][0].transform

        # # Waypoint(Transform(Location(x=-833.713867, y=332.102539, z=348.673737), Rotation(pitch=361.685852, yaw=269.570801, roll=0.000000)))

        # offset_transform = carla.Transform(carla.Location(current_transform.location.x, current_transform.location.y, current_transform.location.z + 10), current_transform.rotation)

        # first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', offset_transform)

        # first_vehicle.

        # spectator.set_transform(offset_transform)


    while(True):
        t = world.get_spectator().get_transform()
        # coordinate_str = "(x,y) = ({},{})".format(t.location.x, t.location.y)
        # coordinate_str = "(x,y,z) = ({},{},{})".format(t.location.x, t.location.y,t.location.z)
        # print (coordinate_str)

        current_location = carla.Location(x=t.location.x, y=t.location.y, z=t.location.z)

        current_waypoint = carla_map.get_waypoint(current_location)

        # mapo = CarlaDataProvider.get_map(current_location)
        #print(mapo)
        print(current_location)
        time.sleep(_SLEEP_TIME_)



if __name__ == '__main__':
	main()


