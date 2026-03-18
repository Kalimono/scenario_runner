#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import copy

import carla
import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorDestroy,
    ActorFlow,
    ActorFlowSections,
    ActorSink,
    ActorTransformSetter,
    ChangeAutoPilot,
    KeepVelocity,
    ScenarioTimeout,
    SpawnActorBatch,
    StopVehicle,
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
from srunner.tools.background_manager import RemoveRoadLane
from srunner.tools.scenario_helper import (
    generate_target_waypoint_list,
    get_opposite_dir_lanes,
    get_waypoint_in_distance,
)


class RuralRelaxation(BasicScenario):
    timeout = 9999  # Timeout of scenario in seconds

    def __init__(
        self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=6000
    ):
        self._map = CarlaDataProvider.get_map()
        self._tm = CarlaDataProvider.get_client().get_trafficmanager(CarlaDataProvider.get_traffic_manager_port())
        self._world = world
        self.timeout = timeout
        self.config = config

        self.loop_frequency = int(self.config.other_parameters["other_vehicles"]["loop_frequency"])

        self.loop_1 = [
            carla.Transform(carla.Location(x=-5.72, y=29.14, z=0.00), carla.Rotation(pitch=0.00, yaw=90.48, roll=0.00)),
            carla.Transform(
                carla.Location(x=30.13, y=63.57, z=0.03), carla.Rotation(pitch=360.00, yaw=1.68, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=85.87, y=33.11, z=0.03), carla.Rotation(pitch=0.00, yaw=-89.31, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=40.12, y=-3.50, z=0.03), carla.Rotation(pitch=0.00, yaw=-178.81, roll=0.00)
            ),
        ]

        self.loop_1_mirrored = self.create_opposite_direction_wp_transforms(self.loop_1)

        self.loop_2 = [
            carla.Transform(
                carla.Location(x=-1.74, y=-28.03, z=0.00), carla.Rotation(pitch=360.00, yaw=270.48, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=-1.26, y=-85.97, z=0.00), carla.Rotation(pitch=360.00, yaw=270.48, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=-0.86, y=-133.85, z=0.00), carla.Rotation(pitch=360.00, yaw=270.48, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=-0.32, y=-198.43, z=0.00), carla.Rotation(pitch=360.00, yaw=270.48, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=23.27, y=-233.32, z=0.94), carla.Rotation(pitch=364.49, yaw=26.33, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=61.76, y=-102.20, z=7.60), carla.Rotation(pitch=355.30, yaw=101.05, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=45.69, y=-3.35, z=0.03), carla.Rotation(pitch=0.00, yaw=-178.31, roll=0.00)
            ),
        ]

        self.loop_2_mirrored = self.create_opposite_direction_wp_transforms(self.loop_2)

        self.loop_3 = [
            carla.Transform(
                carla.Location(x=-139.78, y=-162.49, z=0.14), carla.Rotation(pitch=0.00, yaw=-179.44, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=-198.31, y=-201.31, z=0.00), carla.Rotation(pitch=360.00, yaw=270.02, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=-99.49, y=-204.66, z=7.37), carla.Rotation(pitch=360.55, yaw=363.83, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=-3.82, y=-197.78, z=0.00), carla.Rotation(pitch=0.00, yaw=90.48, roll=0.00)
            ),
        ]

        self.loop_3_mirrored = self.create_opposite_direction_wp_transforms(self.loop_3)

        self.loop_4 = [
            carla.Transform(
                carla.Location(x=-201.40, y=8.71, z=0.00), carla.Rotation(pitch=360.00, yaw=271.07, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=-177.15, y=48.59, z=0.00), carla.Rotation(pitch=0.00, yaw=-179.50, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=-152.39, y=9.76, z=0.10), carla.Rotation(pitch=-0.73, yaw=90.69, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=-174.41, y=-33.64, z=0.08), carla.Rotation(pitch=0.61, yaw=0.44, roll=0.00)
            ),
        ]

        self.loop_4_mirrored = self.create_opposite_direction_wp_transforms(self.loop_4)

        self.loop_5 = [
            carla.Transform(
                carla.Location(x=-149.46, y=110.98, z=0.00), carla.Rotation(pitch=360.00, yaw=186.13, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=-199.63, y=80.20, z=0.00), carla.Rotation(pitch=360.00, yaw=270.65, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=-164.36, y=51.80, z=0.00), carla.Rotation(pitch=360.00, yaw=0.50, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=-125.21, y=52.14, z=0.00), carla.Rotation(pitch=360.00, yaw=0.50, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=-106.69, y=84.72, z=0.72), carla.Rotation(pitch=-0.02, yaw=98.30, roll=0.00)
            ),
        ]

        self.loop_5_mirrored = self.create_opposite_direction_wp_transforms(self.loop_5)

        self.loop_6 = [
            carla.Transform(carla.Location(x=-54.89, y=59.10, z=0.03), carla.Rotation(pitch=0.00, yaw=3.26, roll=0.00)),
            carla.Transform(carla.Location(x=34.02, y=63.68, z=0.03), carla.Rotation(pitch=0.00, yaw=1.68, roll=0.00)),
            carla.Transform(
                carla.Location(x=59.42, y=95.74, z=0.00), carla.Rotation(pitch=360.00, yaw=137.62, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=-31.79, y=120.84, z=0.00), carla.Rotation(pitch=360.00, yaw=177.87, roll=0.00)
            ),
            carla.Transform(
                carla.Location(x=-103.16, y=84.83, z=0.72), carla.Rotation(pitch=359.94, yaw=278.23, roll=0.00)
            ),
        ]

        self.loop_6_mirrored = self.create_opposite_direction_wp_transforms(self.loop_6)

        # self.loop_7 = [
        #     carla.Transform(carla.Location(x=82.33, y=31.14, z=0.03), carla.Rotation(pitch=360.00, yaw=87.56, roll=0.00)),
        #     carla.Transform(carla.Location(x=19.57, y=59.80, z=0.03), carla.Rotation(pitch=0.01, yaw=182.12, roll=0.00)),
        #     carla.Transform(carla.Location(x=-2.22, y=28.52, z=0.00), carla.Rotation(pitch=360.00, yaw=270.48, roll=0.00)),
        #     carla.Transform(carla.Location(x=52.50, y=0.35, z=0.03), carla.Rotation(pitch=360.00, yaw=1.69, roll=0.00))]

        # self.loop_7_mirrored = self.create_opposite_direction_wp_transforms(self.loop_7)

        # self.loop_8 = [
        #     carla.Transform(carla.Location(x=49.49, y=-149.63, z=8.14), carla.Rotation(pitch=364.49, yaw=418.40, roll=0.00)),
        #     carla.Transform(carla.Location(x=55.90, y=-67.53, z=4.72), carla.Rotation(pitch=355.30, yaw=456.99, roll=0.00)),
        #     carla.Transform(carla.Location(x=39.38, y=-3.51, z=0.03), carla.Rotation(pitch=0.00, yaw=-178.90, roll=0.00)),
        #     carla.Transform(carla.Location(x=-1.28, y=-83.88, z=0.00), carla.Rotation(pitch=360.00, yaw=270.48, roll=0.00)),
        #     carla.Transform(carla.Location(x=-0.47, y=-180.32, z=0.00), carla.Rotation(pitch=360.00, yaw=270.48, roll=0.00))]

        # self.loop_8_mirrored = self.create_opposite_direction_wp_transforms(self.loop_8)

        self.transforms_rules_list = [
            (self.loop_1, False, "loop_1"),
            (self.loop_1_mirrored, False, "loop_1_mirrored"),
            (self.loop_2, True, "loop_2"),
            (self.loop_2_mirrored, False, "loop_2_mirrored"),
            (self.loop_3, False, "loop_3"),
            (self.loop_3_mirrored, False, "loop_3_mirrored"),
            (self.loop_4, False, "loop_4"),
            (self.loop_4_mirrored, False, "loop_4_mirrored"),
            (self.loop_5, False, "loop_5"),
            (self.loop_5_mirrored, False, "loop_5_mirrored"),
            (self.loop_6, False, "loop_6"),
            (self.loop_6_mirrored, False, "loop_6_mirrored"),
        ]

        self.start_indicies_dict = {
            "loop_1": [0, 2, 3],
            "loop_1_mirrored": [0, 2, 3],
            "loop_2": [0, 4, 5, 2],
            "loop_2_mirrored": [6, 2, 1, 4],
            "loop_3": [3, 1, 2, 0],
            "loop_3_mirrored": [0, 2, 1, 3],
            "loop_4": [0, 2, 1, 3],
            "loop_4_mirrored": [3, 1, 2, 0],
            "loop_5": [1, 4],
            "loop_5_mirrored": [4, 1, 3],
            "loop_6": [3, 2],
            "loop_6_mirrored": [4, 1, 2],
        }

        # # # self.draw_enumerated_wps(self.loop_5)
        # colors = [carla.Color(255, 0, 0), carla.Color(0, 0, 0), carla.Color(0, 0, 255), carla.Color(255, 0, 255)]

        # color_counter = 0
        # for transform_element in self.transforms_rules_list:
        #     if color_counter >= len(colors):
        #         color_counter = 0

        #     color = colors[color_counter]
        #     self.draw_enumerated_wps(transform_element[0], color)
        #     color_counter += 1

        self.ego_end_transform = carla.Transform(
            carla.Location(x=-10.89, y=-86.68, z=16.89), carla.Rotation(pitch=-85.91, yaw=-87.11, roll=0.00)
        )

        self._attribute_filter = {"base_type": "car", "has_lights": True, "special_type": ""}

        super(RuralRelaxation, self).__init__(
            "RuralRelaxation", ego_vehicles, config, world, debug_mode, criteria_enable=criteria_enable
        )

    def loop_list(self, lst, start_index):
        return lst[start_index:] + lst[:start_index]

    def draw_enumerated_wps(self, wp_transform_list, color):
        wps = self.create_wps_from_transform_list(wp_transform_list)

        for n, wp in enumerate(wps):
            self._world.debug.draw_string(
                wp.transform.location, f"{n}", draw_shadow=False, color=color, life_time=1200.0, persistent_lines=True
            )

    def create_opposite_direction_wp_transforms(self, transform_list):
        transform_list_copy = transform_list.copy()
        transform_list_copy.reverse()
        opposite_wp_transform_list = []
        for transform in transform_list_copy:
            try:
                opposite_wp = get_opposite_dir_lanes(self._map.get_waypoint(transform.location))[0]
            except IndexError:
                print(transform.location)
            opposite_wp_transform_list.append(opposite_wp.transform)
        return opposite_wp_transform_list

    def create_wps_from_transform_list(self, transform_list):
        return [self._map.get_waypoint(transform.location) for transform in transform_list]

    def create_locations_from_transform_list(self, transform_list):
        return [transform.location for transform in transform_list]

    def loop_location_path_list(self, locations, n_iterations):
        extended_location_path_list = locations.copy()

        for i in range(n_iterations):
            extended_location_path_list += locations[1:]

        return extended_location_path_list

    def spawn_actor_with_path(self, transform_list, path, keep_right=False):
        actor = CarlaDataProvider.request_new_actor(
            "vehicle.*", transform_list[0], rolename="scenario", attribute_filter=self._attribute_filter, tick=False
        )

        if actor is None:
            print(transform_list[0].location)
            return

        actor.set_autopilot(True, CarlaDataProvider.get_traffic_manager_port())
        self._tm.set_path(actor, path)

        if keep_right:
            self._tm.keep_right_rule_percentage(actor, 100)
        else:
            self._tm.keep_right_rule_percentage(actor, 0)

        self._tm.update_vehicle_lights(actor, True)

    def _initialize_actors(self, config):
        for transform_list, rule, name in self.transforms_rules_list:
            if len(transform_list) < self.loop_frequency:
                self.loop_frequency = len(transform_list)
            for start_index in self.start_indicies_dict[name][: self.loop_frequency]:
                transform_list = self.loop_list(transform_list, start_index)
                locations = self.create_locations_from_transform_list(transform_list)
                path = self.loop_location_path_list(locations, 10)
                self.spawn_actor_with_path(transform_list, path, rule)

        self.set_traffic_light_times()

    def set_traffic_light_times(self, green_time=6, yellow_time=2, red_time=0):
        traffic_lights = self._world.get_actors().filter("traffic.traffic_light")

        for traffic_light in traffic_lights:
            traffic_light.set_green_time(green_time)
            traffic_light.set_yellow_time(yellow_time)
            traffic_light.set_red_time(red_time)

    def _create_behavior(self):
        test_stretch = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        ego_end = InTriggerDistanceToLocation(self.ego_vehicles[0], self.ego_end_transform.location, 10)

        test_stretch.add_child(ego_end)

        root = py_trees.composites.Sequence()
        root.add_child(test_stretch)

        return root

    def _create_test_criteria(self):
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        self.remove_all_actors()
