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
                                                                      SyncArrival)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill,
                                                                               InTriggerDistanceToLocation)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance, generate_target_waypoint_list, get_same_dir_lanes


class BicycleCollision(BasicScenario):
    timeout = 120

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._ego_vehicle_start = carla.Transform(carla.Location(x=258.476257, y=-185.648209, z=0.019585), carla.Rotation(pitch=0.000000, yaw=-89.823250, roll=0.000000))
        self._ego_vehicle_end = carla.Transform(carla.Location(x=258.826813, y=-299.291046, z=0.019585), carla.Rotation(pitch=0.000000, yaw=-89.823250, roll=0.000000))

        self._crossing_point = carla.Transform(carla.Location(x=303.051575, y=-250.038528, z=0.004333), carla.Rotation(pitch=0.000000, yaw=179.605499, roll=0.000000))

        self._bicycle_start = carla.Transform(carla.Location(x=260.647400, y=-249.746567, z=0.004333), carla.Rotation(pitch=0.000000, yaw=-180.394501, roll=0.000000))

        self._bicycle = None

        super(BicycleCollision, self).__init__("BicycleCollision",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        # for transform in self._initial_actor_transforms:
        #     # bp = CarlaDataProvider.get_world().get_blueprint_library().find('vehicle.audi.a2')
        #     # bp.set_attribute('role_name', 'autopilot')
        self._bicycle = CarlaDataProvider.request_new_actor('vehicle.gazelle.omafiets', self._bicycle_start)
            # vehicle.enable_constant_velocity(carla.Vector3D(7, 0, 0))

            # sensor = self._world.spawn_actor(self._collision_bp, carla.Transform(), attach_to=vehicle)
            # sensor.listen(lambda _: self.stop_constant_velocity())

    def stop_constant_velocity(self):
        """Stops the constant velocity behavior"""
        self._is_constant_velocity_active = False
        for actor in self._actor_list:
            actor.disable_constant_velocity()
            self._tm.ignore_vehicles_percentage(actor, 0)

    def _create_behavior(self):

        ego_end = InTriggerDistanceToLocation(self.ego_vehicles[0], self._ego_vehicle_end.location, distance=5)
        sync_arrival = SyncArrival(self._bicycle, self.ego_vehicles[0], self._crossing_point)

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        
        root.add_child(ego_end)
        root.add_child(sync_arrival)

        return root

    def _create_test_criteria(self):
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        self.remove_all_actors()