#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import os

import carla
import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import PlayMp3File, ScenarioTimeout, SpawnActorBatch
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
from srunner.tools.scenario_helper import generate_target_waypoint_list, get_same_dir_lanes, get_waypoint_in_distance


class UrbanIntersections(BasicScenario):
    timeout = 120  # Timeout of scenario in seconds

    def __init__(
        self,
        world,
        ego_vehicles,
        config,
        randomize=False,
        debug_mode=False,
        criteria_enable=True,
        timeout=6000,
    ):
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout
        self.config = config

        self._world = CarlaDataProvider.get_world()

        self.n_others = int(self.config.other_parameters["other_vehicles"]["n_others"])

        self.dir = os.path.dirname(os.path.abspath(__file__))
        self.assets_path = os.path.join(self.dir.replace("srunner/scenarios", ""), "assets")

        self.skip_spawn_road_id_list = [
            12,
            19,
            20,
            34,
            35,
            36,
            37,
            38,
            2034,
            2035,
            2039,
            2052,
            2043,
            2344,
            2363,
            2343,
            2358,
            2370,
            2353,
        ]

        self.destroy_actor_road_id_list = [
            12,
            34,
            35,
            36,
            37,
            38,
            2034,
            2035,
            2039,
            2052,
            2043,
            2344,
            2363,
            2343,
            2358,
            2370,
            2353,
        ]

        self.sound_instructions_transformations_order = {
            1: [
                "left",
                carla.Transform(
                    carla.Location(x=155.05, y=30.10, z=0.04), carla.Rotation(pitch=360.00, yaw=269.95, roll=0.00)
                ),
            ],
            2: [
                "straight",
                carla.Transform(
                    carla.Location(x=139.65, y=-1.98, z=0.02), carla.Rotation(pitch=360.00, yaw=180.26, roll=0.00)
                ),
            ],
            3: [
                "right",
                carla.Transform(
                    carla.Location(x=67.68, y=-1.64, z=0.00), carla.Rotation(pitch=0.00, yaw=-179.75, roll=0.00)
                ),
            ],
            4: [
                "left",
                carla.Transform(
                    carla.Location(x=32.30, y=-51.20, z=0.00), carla.Rotation(pitch=360.00, yaw=271.53, roll=0.00)
                ),
            ],
            5: [
                "straight",
                carla.Transform(
                    carla.Location(x=-10.63, y=-91.41, z=0.00), carla.Rotation(pitch=0.00, yaw=-179.91, roll=0.00)
                ),
            ],
            6: [
                "straight",
                carla.Transform(
                    carla.Location(x=-85.92, y=-91.52, z=0.00), carla.Rotation(pitch=0.00, yaw=-179.91, roll=0.00)
                ),
            ],
            7: [
                "straight",
                carla.Transform(
                    carla.Location(x=-147.90, y=-91.62, z=0.00), carla.Rotation(pitch=0.00, yaw=-179.91, roll=0.00)
                ),
            ],
            8: [
                "left",
                carla.Transform(
                    carla.Location(x=-271.95, y=-46.16, z=0.00), carla.Rotation(pitch=0.00, yaw=90.42, roll=0.00)
                ),
            ],
            9: [
                "right",
                carla.Transform(
                    carla.Location(x=-228.06, y=3.10, z=0.00), carla.Rotation(pitch=360.00, yaw=359.86, roll=0.00)
                ),
            ],
            10: [
                "left",
                carla.Transform(
                    carla.Location(x=-191.56, y=47.86, z=0.06), carla.Rotation(pitch=360.00, yaw=89.98, roll=0.00)
                ),
            ],
            11: [
                "right",
                carla.Transform(
                    carla.Location(x=-167.01, y=91.38, z=0.00), carla.Rotation(pitch=360.00, yaw=0.07, roll=0.00)
                ),
            ],
            12: [
                "right",
                carla.Transform(
                    carla.Location(x=-127.23, y=111.51, z=0.00), carla.Rotation(pitch=360.00, yaw=89.49, roll=0.00)
                ),
            ],
            13: [
                "left",
                carla.Transform(
                    carla.Location(x=-185.89, y=128.79, z=0.06), carla.Rotation(pitch=0.00, yaw=248.11, roll=0.00)
                ),
            ],
            14: [
                "right",
                carla.Transform(
                    carla.Location(x=-268.73, y=44.81, z=0.00), carla.Rotation(pitch=0.00, yaw=-90.58, roll=0.00)
                ),
            ],
            15: [
                "left",
                carla.Transform(
                    carla.Location(x=-227.50, y=3.10, z=0.00), carla.Rotation(pitch=360.00, yaw=359.86, roll=0.00)
                ),
            ],
            16: [
                "left",
                carla.Transform(
                    carla.Location(x=-188.08, y=-48.12, z=0.06), carla.Rotation(pitch=0.00, yaw=-90.02, roll=0.00)
                ),
            ],
            17: [
                "straight",
                carla.Transform(
                    carla.Location(x=-271.95, y=-46.29, z=0.00), carla.Rotation(pitch=0.00, yaw=90.42, roll=0.00)
                ),
            ],
            18: [
                "left",
                carla.Transform(
                    carla.Location(x=-229.22, y=91.30, z=0.00), carla.Rotation(pitch=360.00, yaw=0.07, roll=0.00)
                ),
            ],
            # 19: [
            #     "right",
            #     carla.Transform(
            #         carla.Location(x=-184.56, y=39.53, z=0.06), carla.Rotation(pitch=0.00, yaw=-90.02, roll=0.00)
            #     ),
            # ],
            # 20: [
            #     "straight",
            #     carla.Transform(
            #         carla.Location(x=-168.21, y=2.96, z=0.00), carla.Rotation(pitch=360.00, yaw=359.86, roll=0.00)
            #     ),
            # ],
            # 21: [
            #     "straight",
            #     carla.Transform(
            #         carla.Location(x=-86.84, y=2.76, z=0.00), carla.Rotation(pitch=360.00, yaw=359.86, roll=0.00)
            #     ),
            # ],
            # 22: [
            #     "right",
            #     carla.Transform(
            #         carla.Location(x=-8.01, y=2.57, z=0.00), carla.Rotation(pitch=360.00, yaw=359.86, roll=0.00)
            #     ),
            # ],
            # 23: [
            #     "straight",
            #     carla.Transform(
            #         carla.Location(x=28.09, y=50.14, z=0.00), carla.Rotation(pitch=360.00, yaw=90.02, roll=0.00)
            #     ),
            # ],
            # 24: [
            #     "left",
            #     carla.Transform(
            #         carla.Location(x=28.07, y=105.25, z=0.00), carla.Rotation(pitch=360.00, yaw=90.02, roll=0.00)
            #     ),
            # ],
        }

        # self.ego_end_transform = carla.Transform(
        #     carla.Location(x=28.07, y=107.67, z=0.00), carla.Rotation(pitch=360.00, yaw=90.02, roll=0.00)
        # )

        self.ego_end_transform = carla.Transform(
            carla.Location(x=-184.56, y=39.53, z=0.06), carla.Rotation(pitch=360.00, yaw=359.86, roll=0.00)
        )

        self.draw_enumerated_wps(
            [i[1] for i in self.sound_instructions_transformations_order.values()], carla.Color(255, 0, 0)
        )
        # colors = [carla.Color(255, 0, 0), carla.Color(0, 0, 0), carla.Color(0, 0, 255), carla.Color(255, 0, 255)]

        # color_counter = 0
        # for transform_element in self.sound_instructions_transformations_order.values():
        #     if color_counter >= len(colors):
        #         color_counter = 0

        #     color = colors[color_counter]
        #     self.draw_enumerated_wps(transform_element[1], color)
        #     color_counter += 1

        super(UrbanIntersections, self).__init__(
            "UrbanIntersections",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    def create_wps_from_transform_list(self, transform_list):
        return [self._map.get_waypoint(transform.location) for transform in transform_list]

    def draw_enumerated_wps(self, wp_transform_list, color):
        wps = self.create_wps_from_transform_list(wp_transform_list)

        for n, wp in enumerate(wps):
            self._world.debug.draw_string(
                wp.transform.location, f"{n}", draw_shadow=False, color=color, life_time=1200.0, persistent_lines=True
            )

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        self.set_traffic_light_times()

    def set_traffic_light_times(self, green_time=6, yellow_time=2, red_time=0):
        traffic_lights = self._world.get_actors().filter("traffic.traffic_light")

        for traffic_light in traffic_lights:
            traffic_light.set_green_time(green_time)
            traffic_light.set_yellow_time(yellow_time)
            traffic_light.set_red_time(red_time)

    def _create_behavior(self):
        actor_batch_spawn = SpawnActorBatch(
            self.n_others, self.skip_spawn_road_id_list, self.destroy_actor_road_id_list
        )

        ego_end = InTriggerDistanceToLocation(self.ego_vehicles[0], self.ego_end_transform.location, 10)

        instruction_sequence = py_trees.composites.Sequence()

        for key, value in self.sound_instructions_transformations_order.items():
            current_sequence = py_trees.composites.Sequence()
            current_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0], value[1].location, 10))

            current_sequence.add_child(
                PlayMp3File(
                    f"{self.assets_path}/{value[0]}.mp3",
                )
            )

            instruction_sequence.add_child(current_sequence)

        instruction_sequence.add_child(ego_end)

        main_sequence = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
        )

        main_sequence.add_child(actor_batch_spawn)
        main_sequence.add_child(instruction_sequence)

        root = py_trees.composites.Sequence()

        root.add_child(main_sequence)

        return root

    def _create_test_criteria(self):
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        self.remove_all_actors()
