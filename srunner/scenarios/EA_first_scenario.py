#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import carla
import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorDestroy,
    ActorFlow,
    ActorSink,
    ActorTransformSetter,
    BicycleFlow,
    ChangeAutoPilot,
    KeepVelocity,
    StopVehicle,
    SyncArrivalWithOther,
    WaypointFollower,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance,
    InTriggerDistanceToLocation,
    InTriggerDistanceToNextIntersection,
    InTriggerDistanceToVehicle,
    StandStill,
)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import generate_target_waypoint_list, get_same_dir_lanes, get_waypoint_in_distance


class EA_first_scenario(BasicScenario):
    timeout = 9999999

    def __init__(
        self,
        world,
        ego_vehicles,
        config,
        randomize=False,
        debug_mode=False,
        criteria_enable=True,
        timeout=9999999,
    ):
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self.world = CarlaDataProvider.get_world()

        spectator_position = carla.Transform(
            carla.Location(x=183.32, y=-233.74, z=22.35),
            carla.Rotation(pitch=-30.93, yaw=-39.47, roll=0.00),
        )

        spectator = world.get_spectator()
        spectator.set_transform(spectator_position)

        self._arrival_event_trigger = carla.Transform(
            carla.Location(x=233.62, y=-249.56, z=0.00), carla.Rotation(pitch=0.00, yaw=179.61, roll=0.00)
        )

        self._ego_path = [
            carla.Transform(
                carla.Location(x=199.02, y=-248.87, z=0.04), carla.Rotation(pitch=0.06, yaw=177.05, roll=0.00)
            )
        ]

        self._ego_end = carla.Transform(
            carla.Location(x=159.48, y=-240.05, z=0.04), carla.Rotation(pitch=0.00, yaw=-209.81, roll=0.00)
        )

        self._bicycle_start = carla.Transform(
            carla.Location(x=208.35, y=-272.96, z=0.02), carla.Rotation(pitch=360.00, yaw=90.31, roll=0.00)
        )

        self._bicycle_end = carla.Transform(
            carla.Location(x=207.70, y=-238.45, z=0.02), carla.Rotation(pitch=360.00, yaw=107.02, roll=0.00)
        )

        super(EA_first_scenario, self).__init__(
            "EA_first_scenario",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    def _initialize_actors(self, config):
        self._bicycle = CarlaDataProvider.request_new_actor("vehicle.gazelle.omafiets", self._bicycle_start)

        for actor in self.world.get_actors():
            if isinstance(actor, carla.TrafficLight):
                actor.set_state(carla.TrafficLightState.Green)
                actor.set_green_time(1000.0)

    def stop_constant_velocity(self):
        """Stops the constant velocity behavior"""
        self._is_constant_velocity_active = False
        for actor in self._actor_list:
            actor.disable_constant_velocity()
            self._tm.ignore_vehicles_percentage(actor, 0)

    def _create_behavior(self):
        pre_event = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        arrival_event_trigger = InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            self._arrival_event_trigger.location,
            distance=5.0,
        )

        pre_event.add_child(arrival_event_trigger)

        sync_arrival_other = SyncArrivalWithOther(
            self._bicycle,
            self.ego_vehicles[0],
            self._bicycle_end,
            self._ego_path,
            end_dist=1.0,
            reference_dist_offset=1.5,
        )

        ego_end = InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            self._ego_end.location,
            distance=5.0,
        )

        arrival_event = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        arrival_event.add_child(ego_end)
        arrival_event.add_child(sync_arrival_other)

        root = py_trees.composites.Sequence()

        root.add_child(pre_event)
        root.add_child(arrival_event)

        return root

    def _create_test_criteria(self):
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        self.remove_all_actors()
