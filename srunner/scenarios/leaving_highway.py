#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower,
                                                                      ActorSink,
                                                                      ChangeAutoPilot,
                                                                      ActorFlow,
                                                                      ScenarioTimeout,
                                                                      ActorFlowSafe,
                                                                      )
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill,
                                                                               InTriggerDistanceToLocation)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance, generate_target_waypoint_list, get_same_dir_lanes
from srunner.tools.background_manager import RemoveRoadLane


class LeavingHighway(BasicScenario):
    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout
        self.config = config

        self._second_lane_start = carla.Transform(carla.Location(x=287.214508, y=37.715034, z=2.104192), carla.Rotation(pitch=357.493896, yaw=0.977936, roll=0.000000))
        self._second_lane_end = carla.Transform(carla.Location(x=-262.951996, y=407.801849, z=0.000000), carla.Rotation(pitch=360.000000, yaw=179.791809, roll=0.000000))
        self._last_lane_start = carla.Transform(carla.Location(x=285.024139, y=30.676624, z=2.206451), carla.Rotation(pitch=357.433716, yaw=0.977936, roll=0.000000))
        self._last_lane_end = carla.Transform(carla.Location(x=-225.769119, y=414.666809, z=0.000000), carla.Rotation(pitch=360.000000, yaw=179.791809, roll=0.000000))
        self._first_lane_start = carla.Transform(carla.Location(x=290.787109, y=41.276527, z=1.948289), carla.Rotation(pitch=357.588531, yaw=0.977936, roll=0.000000))
        self._first_lane_end = carla.Transform(carla.Location(x=-225.945053, y=404.167358, z=0.000000), carla.Rotation(pitch=360.000000, yaw=179.791809, roll=0.000000))

        self._ego_vehicle_start = carla.Transform(carla.Location(x=387.193573, y=84.802063, z=0.000000), carla.Rotation(pitch=0.000000, yaw=-61.237587, roll=0.000000))
        self._ego_vehicle_end = carla.Transform(carla.Location(x=-124.726662, y=99.898415, z=5.371345), carla.Rotation(pitch=364.798492, yaw=260.301422, roll=0.000000))

        self._route_planner = CarlaDataProvider.get_global_route_planner()

        super(LeavingHighway, self).__init__("LeavingHighway",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

    def stop_constant_velocity(self):
        """Stops the constant velocity behavior"""
        self._is_constant_velocity_active = False
        for actor in self._actor_list:
            actor.disable_constant_velocity()
            self._tm.ignore_vehicles_percentage(actor, 0)

    def _create_behavior(self):
        start_wp_second = get_same_dir_lanes(self._map.get_waypoint(self._second_lane_start.location))[2]
        end_wp_second = get_same_dir_lanes(self._map.get_waypoint(self._second_lane_end.location))[2]

        traffic_flow_second = ActorFlowSafe(start_wp_second, end_wp_second, [80, 90], astar=False, actor_speed=25, initial_actors=True)#, initial_junction=True)
        
        start_wp_forth = get_same_dir_lanes(self._map.get_waypoint(self._last_lane_start.location))[0]
        end_wp_forth = get_same_dir_lanes(self._map.get_waypoint(self._last_lane_end.location))[0]

        traffic_flow_forth = ActorFlowSafe(start_wp_forth, end_wp_forth, [30, 60], actor_speed=25, initial_actors=True)#, initial_junction=True)

        start_wp_first = get_same_dir_lanes(self._map.get_waypoint(self._first_lane_start.location))[3]
        end_wp_first = get_same_dir_lanes(self._map.get_waypoint(self._first_lane_end.location))[3]

        traffic_flow_first = ActorFlowSafe(start_wp_first, end_wp_first, [30, 60], actor_speed=25)#, initial_actors=True)#, initial_junction=True)
        
        ego_end_first = InTriggerDistanceToLocation(self.ego_vehicles[0], self._ego_vehicle_end.location, distance=20)

        first_stretch = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # first_stretch.add_child(traffic_flow_second)

        # first_stretch.add_child(traffic_flow_forth)

        first_stretch.add_child(traffic_flow_first)

        # parallel_root.add_child(traffic_flow_first) Location(x=147.611847, y=35.332066, z=9.436032)

        first_stretch.add_child(ego_end_first)
        first_stretch.add_child(ScenarioTimeout(self.timeout, self.config.name))

        second_stretch = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        ego_end_second = InTriggerDistanceToLocation(self.ego_vehicles[0], carla.Location(x=147.611847, y=35.332066, z=9.436032), distance=20)

        second_stretch.add_child(ego_end_second)

        root = py_trees.composites.Sequence()
        root.add_child(first_stretch)
        root.add_child(second_stretch)


        return root

    def _create_test_criteria(self):
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        self.remove_all_actors()