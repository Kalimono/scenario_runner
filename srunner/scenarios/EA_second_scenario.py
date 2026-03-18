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
    DriveToLocation,
    KeepVelocity,
    StopVehicle,
    SyncArrivalWithOther,
    UpdateConstantVeloctiy,
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


class EA_second_scenario(BasicScenario):
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
            carla.Location(x=277.32, y=-273.74, z=31.73),
            carla.Rotation(pitch=-43.93, yaw=119.47, roll=-0.00),
        )

        spectator = world.get_spectator()
        spectator.set_transform(spectator_position)

        self._arrival_event_trigger = carla.Transform(
            carla.Location(x=258.52, y=-200.75, z=0.02), carla.Rotation(pitch=0.00, yaw=-89.82, roll=0.00)
        )

        self._ego_path = [
            carla.Transform(
                carla.Location(x=258.68, y=-252.08, z=0.02), carla.Rotation(pitch=0.00, yaw=-89.82, roll=0.00)
            )
        ]

        self._ego_end = carla.Transform(
            carla.Location(x=258.82, y=-297.68, z=0.02), carla.Rotation(pitch=0.00, yaw=-89.82, roll=0.00)
        )

        self._bicycle_start = carla.Transform(
            carla.Location(x=301.24, y=-242.64, z=0.00), carla.Rotation(pitch=0.00, yaw=180.61, roll=0.00)
        )

        self._bicycle_end = carla.Transform(
            carla.Location(x=232.07, y=-242.16, z=0.00), carla.Rotation(pitch=360.00, yaw=359.61, roll=0.00)
        )  # carla.Transform(carla.Location(x=247.29, y=-242.27, z=0.00), carla.Rotation(pitch=0.00, yaw=179.61, roll=0.00))

        self._other_car_start = carla.Transform(
            carla.Location(x=215.17, y=-245.93, z=0.00), carla.Rotation(pitch=360.00, yaw=359.61, roll=0.00)
        )

        self._other_car_end = carla.Transform(
            carla.Location(x=334.12, y=-246.75, z=0.00), carla.Rotation(pitch=360.00, yaw=359.61, roll=0.00)
        )  # carla.Transform(carla.Location(x=286.58, y=-246.43, z=0.00), carla.Rotation(pitch=360.00, yaw=359.61, roll=0.00))

        super(EA_second_scenario, self).__init__(
            "EA_second_scenario",
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

        self._other_car = CarlaDataProvider.request_new_actor("vehicle.mercedes.sprinter", self._other_car_start)
        self._other_car.set_autopilot(False)

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

        other_car_drive = DriveToLocation(self._other_car_end.location, self._other_car)

        pre_event.add_child(arrival_event_trigger)

        sync_arrival_other = SyncArrivalWithOther(
            self._bicycle,
            self._other_car,
            self._bicycle_end,
            [self._other_car_end],
            end_dist=1.0,
            reference_dist_offset=-5.0,
        )

        ego_end = InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            self._ego_end.location,
            distance=5.0,
        )

        other_car_update_veloctiy = UpdateConstantVeloctiy(self._other_car)

        # other_drive = WaypointFollower(
        #     self._other_car,
        #     target_speed=30.0,
        #     plan=[self._other_car_end])

        arrival_event = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        arrival_event.add_child(ego_end)
        arrival_event.add_child(sync_arrival_other)
        arrival_event.add_child(other_car_drive)
        arrival_event.add_child(other_car_update_veloctiy)
        # arrival_event.add_child(other_drive)

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
