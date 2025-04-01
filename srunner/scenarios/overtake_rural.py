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
                                                                      ActorFlowSafe)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill,
                                                                               InTriggerDistanceToLocation)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance, generate_target_waypoint_list


class OvertakeRural(BasicScenario):
    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._vehicle_oncoming_spawn_wps = []
        self._vehicle_front_spawn_wps = []

        self._oncoming_vehicles = []
        self._front_vehicles = []

        self._oncoming_traffic_start_1 = carla.Transform(carla.Location(x=-804.468079, y=-1204.142212, z=342.534973), carla.Rotation(pitch=358.680420, yaw=269.905579, roll=0.000000))
        self._oncoming_traffic_end_1 = carla.Transform(carla.Location(x=2368.949707, y=-1586.074585, z=336.887268), carla.Rotation(pitch=360.270416, yaw=252.085724, roll=0.000000))

        self._oncoming_traffic_start_2 = carla.Transform(carla.Location(x=2372.415283, y=-1589.648315, z=336.879913), carla.Rotation(pitch=358.680420, yaw=269.905579, roll=0.000000))
        self._oncoming_traffic_end_2 = carla.Transform(carla.Location(x=3612.987793, y=-94.270981, z=322.756348), carla.Rotation(pitch=360.270416, yaw=252.085724, roll=0.000000))

        self._oncoming_traffic_start_3 = carla.Transform(carla.Location(x=3628.272705, y=-90.772179, z=323.037567), carla.Rotation(pitch=358.680420, yaw=269.905579, roll=0.000000))
        self._oncoming_traffic_end_3 = carla.Transform(carla.Location(x=3815.438721, y=1502.190063, z=350.884430), carla.Rotation(pitch=360.270416, yaw=252.085724, roll=0.000000))

        self._oncoming_traffic_start_4 = carla.Transform(carla.Location(x=3815.715820, y=1505.689575, z=350.869263), carla.Rotation(pitch=358.680420, yaw=269.905579, roll=0.000000))
        self._oncoming_traffic_end_4 = carla.Transform(carla.Location(x=3907.308838, y=2957.010742, z=369.621643), carla.Rotation(pitch=360.270416, yaw=252.085724, roll=0.000000))


        self._front_traffic_start = carla.Transform(carla.Location(x=3946.853760, y=2700.881104, z=371.067535), carla.Rotation(pitch=358.680420, yaw=269.905579, roll=0.000000))
        self._front_traffic_end = carla.Transform(carla.Location(x=-746.862854, y=-1364.111694, z=342.694519), carla.Rotation(pitch=360.270416, yaw=252.085724, roll=0.000000))

        self._ego_vehicle_start = carla.Transform(carla.Location(x=-836.937561, y=-176.768463, z=341.036835), carla.Rotation(pitch=359.327118, yaw=269.905579, roll=0.000000))
        self._ego_vehicle_end = carla.Transform(carla.Location(x=-746.862854, y=-1364.111694, z=342.694519), carla.Rotation(pitch=360.270416, yaw=252.085724, roll=0.000000))



        self._route_planner = CarlaDataProvider.get_global_route_planner()
        

        super(OvertakeRural, self).__init__("OvertakeRural",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

    def _create_behavior(self):
        oncoming_flow_1 = ActorFlowSafe(self._map.get_waypoint(self._oncoming_traffic_start_1.location), self._map.get_waypoint(self._oncoming_traffic_end_1.location), [180, 200], actor_speed=25)
        oncoming_flow_2 = ActorFlowSafe(self._map.get_waypoint(self._oncoming_traffic_start_2.location), self._map.get_waypoint(self._oncoming_traffic_end_2.location), [180, 200], actor_speed=25)
        oncoming_flow_3 = ActorFlowSafe(self._map.get_waypoint(self._oncoming_traffic_start_3.location), self._map.get_waypoint(self._oncoming_traffic_end_3.location), [180, 200], actor_speed=25)
        oncoming_flow_4 = ActorFlowSafe(self._map.get_waypoint(self._oncoming_traffic_start_4.location), self._map.get_waypoint(self._oncoming_traffic_end_4.location), [180, 200], actor_speed=25)
        front_flow = ActorFlowSafe(self._map.get_waypoint(self._front_traffic_start.location), self._map.get_waypoint(self._front_traffic_end.location), [120, 140], actor_speed=30)

        ego_end = InTriggerDistanceToLocation(self.ego_vehicles[0], self._ego_vehicle_end.location, distance=20)

        parallel_root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        parallel_root.add_child(oncoming_flow_1)
        parallel_root.add_child(oncoming_flow_2)
        parallel_root.add_child(oncoming_flow_3)
        parallel_root.add_child(oncoming_flow_4)
        parallel_root.add_child(front_flow)

        parallel_root.add_child(ego_end)

        scenario_sequence = py_trees.composites.Sequence()

        scenario_sequence.add_child(parallel_root)

        return scenario_sequence

    def _create_test_criteria(self):
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        self.remove_all_actors()