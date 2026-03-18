#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import os

import carla
import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorFlowSections,
    PlayMp3File,
    TrafficLightControllerSetter,
    WaitForever,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToLocation
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_same_dir_lanes


class LeavingHighway(BasicScenario):
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
        self._tm = CarlaDataProvider.get_client().get_trafficmanager(CarlaDataProvider.get_traffic_manager_port())

        self.dir = os.path.dirname(os.path.abspath(__file__))
        self.assets_path = os.path.join(self.dir.replace("srunner/scenarios", ""), "assets")

        self.world = CarlaDataProvider.get_world()

        self.first_lane_speed = int(self.config.other_parameters["other_vehicles"]["first_lane_speed"])
        self.second_lane_speed = int(self.config.other_parameters["other_vehicles"]["second_lane_speed"])
        self.third_lane_speed = int(self.config.other_parameters["other_vehicles"]["fourth_lane_speed"])

        spectator_position = carla.Transform(
            carla.Location(x=473.73, y=-25.74, z=45.35),
            carla.Rotation(pitch=-14.93, yaw=157.40, roll=0.00),
        )

        spectator = world.get_spectator()
        spectator.set_transform(spectator_position)

        self.wp_transforms_ff = [
            carla.Transform(
                carla.Location(x=141.67, y=39.04, z=12.23),
                carla.Rotation(pitch=-15.68, yaw=-1.74, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=237.31, y=40.19, z=6.89),
                carla.Rotation(pitch=-5.70, yaw=0.56, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=334.15, y=41.87, z=2.39),
                carla.Rotation(pitch=-3.86, yaw=0.36, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=369.93, y=36.06, z=2.34),
                carla.Rotation(pitch=-1.89, yaw=-24.57, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=397.66, y=14.42, z=1.88),
                carla.Rotation(pitch=-4.95, yaw=-48.63, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=410.18, y=-13.96, z=1.57),
                carla.Rotation(pitch=-0.61, yaw=-75.94, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=412.20, y=-52.39, z=3.22),
                carla.Rotation(pitch=-1.79, yaw=-87.72, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=412.64, y=-130.78, z=1.95),
                carla.Rotation(pitch=-3.92, yaw=-88.16, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=413.72, y=-207.70, z=1.88),
                carla.Rotation(pitch=-0.01, yaw=-89.16, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=411.35, y=-261.35, z=1.88),
                carla.Rotation(pitch=1.63, yaw=-101.13, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=388.61, y=-319.98, z=1.75),
                carla.Rotation(pitch=-3.95, yaw=-121.64, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=342.57, y=-367.79, z=1.42),
                carla.Rotation(pitch=2.65, yaw=-147.71, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=286.22, y=-391.63, z=1.87),
                carla.Rotation(pitch=-4.01, yaw=-166.88, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=215.56, y=-395.61, z=1.71),
                carla.Rotation(pitch=-2.30, yaw=-179.96, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=139.81, y=-394.57, z=2.09),
                carla.Rotation(pitch=-1.15, yaw=173.19, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=94.50, y=-383.83, z=1.97),
                carla.Rotation(pitch=-3.89, yaw=153.42, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=50.50, y=-358.47, z=2.23),
                carla.Rotation(pitch=-2.41, yaw=138.62, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=13.81, y=-318.81, z=1.73),
                carla.Rotation(pitch=-1.95, yaw=125.46, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-9.90, y=-266.44, z=2.39),
                carla.Rotation(pitch=-1.79, yaw=105.94, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-16.59, y=-207.80, z=2.39),
                carla.Rotation(pitch=-4.64, yaw=91.38, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-16.31, y=-124.87, z=1.87),
                carla.Rotation(pitch=-1.57, yaw=90.36, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-16.08, y=-68.22, z=2.01),
                carla.Rotation(pitch=-2.50, yaw=91.23, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-16.01, y=-11.06, z=2.11),
                carla.Rotation(pitch=-2.68, yaw=89.89, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-15.64, y=58.06, z=1.78),
                carla.Rotation(pitch=-0.32, yaw=90.29, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-15.52, y=90.81, z=2.04),
                carla.Rotation(pitch=0.30, yaw=92.41, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-15.71, y=119.93, z=1.69),
                carla.Rotation(pitch=-1.83, yaw=84.42, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-15.16, y=193.34, z=1.84),
                carla.Rotation(pitch=-2.08, yaw=88.84, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-16.27, y=243.61, z=1.90),
                carla.Rotation(pitch=-0.41, yaw=97.70, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-31.72, y=297.94, z=1.65),
                carla.Rotation(pitch=0.03, yaw=114.47, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-63.79, y=345.52, z=2.24),
                carla.Rotation(pitch=3.24, yaw=132.17, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-100.58, y=376.40, z=1.90),
                carla.Rotation(pitch=-1.51, yaw=152.87, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-151.32, y=397.82, z=1.81),
                carla.Rotation(pitch=-0.19, yaw=163.77, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-196.55, y=404.00, z=1.89),
                carla.Rotation(pitch=-2.63, yaw=178.34, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-243.83, y=404.48, z=1.90),
                carla.Rotation(pitch=-2.71, yaw=-179.34, roll=0.00),
            ),
        ]

        self.first_lane_ff_wps = [
            get_same_dir_lanes(self._map.get_waypoint(i.location))[3] for i in self.wp_transforms_ff
        ]
        self.second_lane_ff_wps = [
            get_same_dir_lanes(self._map.get_waypoint(i.location))[2] for i in self.wp_transforms_ff
        ]
        self.third_lane_ff_wps = [
            get_same_dir_lanes(self._map.get_waypoint(i.location))[1] for i in self.wp_transforms_ff
        ]
        self.fourth_lane_ff_wps = [
            get_same_dir_lanes(self._map.get_waypoint(i.location))[0] for i in self.wp_transforms_ff
        ]

        self.first_lane_locations = [i.transform.location for i in self.first_lane_ff_wps]
        self.second_lane_locations = [i.transform.location for i in self.second_lane_ff_wps]
        self.third_lane_locations = [i.transform.location for i in self.third_lane_ff_wps]
        self.fourth_lane_locations = [i.transform.location for i in self.fourth_lane_ff_wps]

        self.ego_end_first_stretch = carla.Transform(
            carla.Location(x=-34.60, y=133.70, z=0.00), carla.Rotation(pitch=360.07, yaw=140.74, roll=0.00)
        )

        self.wp_transforms_sf = [
            carla.Transform(
                carla.Location(x=-338.54, y=37.36, z=2.49),
                carla.Rotation(pitch=-6.60, yaw=-5.54, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-245.59, y=36.68, z=5.29),
                carla.Rotation(pitch=-0.77, yaw=-3.28, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-166.61, y=37.03, z=9.35),
                carla.Rotation(pitch=-12.55, yaw=-1.44, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-94.73, y=36.83, z=12.33),
                carla.Rotation(pitch=-6.81, yaw=2.97, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=1.34, y=37.41, z=13.40),
                carla.Rotation(pitch=0.34, yaw=2.88, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=65.27, y=39.13, z=12.54),
                carla.Rotation(pitch=1.39, yaw=-7.03, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=97.24, y=38.13, z=12.27),
                carla.Rotation(pitch=-2.27, yaw=-6.46, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=213.42, y=39.23, z=8.28),
                carla.Rotation(pitch=-4.23, yaw=1.57, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=321.17, y=41.53, z=2.42),
                carla.Rotation(pitch=0.57, yaw=0.51, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=360.25, y=39.43, z=1.92),
                carla.Rotation(pitch=-10.92, yaw=-6.42, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=381.37, y=30.30, z=1.96),
                carla.Rotation(pitch=-2.00, yaw=-35.94, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=399.27, y=11.60, z=2.26),
                carla.Rotation(pitch=-9.91, yaw=-55.64, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=408.97, y=-8.97, z=2.11),
                carla.Rotation(pitch=-4.78, yaw=-70.33, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=412.24, y=-50.67, z=1.73),
                carla.Rotation(pitch=-2.91, yaw=-90.74, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=413.02, y=-135.95, z=1.53),
                carla.Rotation(pitch=-1.57, yaw=-87.61, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=413.68, y=-217.25, z=2.11),
                carla.Rotation(pitch=-0.79, yaw=-88.80, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=410.02, y=-268.24, z=1.49),
                carla.Rotation(pitch=-0.46, yaw=-100.45, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=392.10, y=-314.41, z=1.58),
                carla.Rotation(pitch=-6.23, yaw=-119.13, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=360.19, y=-353.79, z=1.83),
                carla.Rotation(pitch=-2.48, yaw=-143.72, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=316.04, y=-382.35, z=1.53),
                carla.Rotation(pitch=2.66, yaw=-162.88, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=264.70, y=-394.96, z=1.81),
                carla.Rotation(pitch=0.85, yaw=-175.04, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=200.76, y=-395.69, z=1.97),
                carla.Rotation(pitch=-1.66, yaw=179.66, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=136.36, y=-394.50, z=2.00),
                carla.Rotation(pitch=-0.83, yaw=171.46, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=88.66, y=-381.56, z=1.76),
                carla.Rotation(pitch=-4.61, yaw=154.10, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=43.69, y=-353.20, z=1.86),
                carla.Rotation(pitch=-4.99, yaw=135.41, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=11.26, y=-315.44, z=1.84),
                carla.Rotation(pitch=-4.20, yaw=121.57, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-3.37, y=-287.11, z=1.94),
                carla.Rotation(pitch=-1.17, yaw=111.19, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-12.61, y=-257.44, z=2.05),
                carla.Rotation(pitch=-5.64, yaw=98.83, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-16.63, y=-220.80, z=1.66),
                carla.Rotation(pitch=-8.80, yaw=92.93, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-16.84, y=-162.38, z=1.04),
                carla.Rotation(pitch=2.77, yaw=87.48, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-16.22, y=-86.81, z=1.89),
                carla.Rotation(pitch=-0.86, yaw=89.91, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-15.79, y=42.74, z=2.65),
                carla.Rotation(pitch=-1.10, yaw=90.15, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-15.51, y=83.25, z=2.02),
                carla.Rotation(pitch=-3.67, yaw=87.68, roll=0.00),
            ),
        ]

        self.first_lane_sf_wps = [
            get_same_dir_lanes(self._map.get_waypoint(i.location))[3] for i in self.wp_transforms_sf
        ]
        self.second_lane_sf_wps = [
            get_same_dir_lanes(self._map.get_waypoint(i.location))[2] for i in self.wp_transforms_sf
        ]
        self.third_lane_sf_wps = [
            get_same_dir_lanes(self._map.get_waypoint(i.location))[1] for i in self.wp_transforms_sf
        ]
        self.fourth_lane_sf_wps = [
            get_same_dir_lanes(self._map.get_waypoint(i.location))[0] for i in self.wp_transforms_sf
        ]

        self.first_lane_sf_locations = [i.transform.location for i in self.first_lane_sf_wps]
        self.second_lane_sf_locations = [i.transform.location for i in self.second_lane_sf_wps]
        self.third_lane_sf_locations = [i.transform.location for i in self.third_lane_sf_wps]
        self.fourth_lane_sf_locations = [i.transform.location for i in self.fourth_lane_sf_wps]

        self.ego_end_second_stretch = carla.Transform(
            carla.Location(x=-132.48, y=-141.07, z=1.86),
            carla.Rotation(pitch=4.69, yaw=148.62, roll=-0.00),
        )

        self.wp_transforms_tf = [
            carla.Transform(
                carla.Location(x=-103.82, y=5.89, z=11.08),
                carla.Rotation(pitch=1.15, yaw=175.55, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-207.76, y=6.03, z=6.77),
                carla.Rotation(pitch=-6.54, yaw=-179.34, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-308.03, y=5.64, z=2.75),
                carla.Rotation(pitch=-5.33, yaw=178.69, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-362.07, y=5.63, z=1.49),
                carla.Rotation(pitch=-4.45, yaw=-179.64, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-397.74, y=6.16, z=1.65),
                carla.Rotation(pitch=-1.06, yaw=-179.39, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-450.08, y=10.39, z=1.78),
                carla.Rotation(pitch=-0.05, yaw=159.79, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-488.21, y=33.68, z=1.77),
                carla.Rotation(pitch=-5.30, yaw=135.92, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-509.99, y=71.05, z=1.96),
                carla.Rotation(pitch=-4.00, yaw=102.89, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-513.67, y=117.05, z=1.82),
                carla.Rotation(pitch=2.18, yaw=89.29, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-514.28, y=194.71, z=1.85),
                carla.Rotation(pitch=-2.48, yaw=90.94, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-514.59, y=256.31, z=1.96),
                carla.Rotation(pitch=-1.76, yaw=85.55, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-502.32, y=309.93, z=1.78),
                carla.Rotation(pitch=-5.25, yaw=72.03, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-476.20, y=357.73, z=1.59),
                carla.Rotation(pitch=-3.64, yaw=55.31, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-438.19, y=396.19, z=1.36),
                carla.Rotation(pitch=0.08, yaw=37.04, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-387.43, y=423.67, z=1.55),
                carla.Rotation(pitch=7.69, yaw=17.56, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-323.16, y=435.67, z=2.02),
                carla.Rotation(pitch=-3.25, yaw=0.27, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-234.46, y=435.50, z=1.95),
                carla.Rotation(pitch=-2.17, yaw=0.65, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-163.49, y=432.25, z=1.77),
                carla.Rotation(pitch=3.00, yaw=-10.17, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-104.93, y=413.71, z=1.58),
                carla.Rotation(pitch=3.19, yaw=-16.72, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-60.81, y=385.96, z=1.79),
                carla.Rotation(pitch=-10.60, yaw=-36.99, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-31.14, y=355.39, z=1.51),
                carla.Rotation(pitch=-2.62, yaw=-49.69, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=-6.39, y=316.83, z=1.60),
                carla.Rotation(pitch=-5.34, yaw=-61.28, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=12.62, y=261.50, z=1.70),
                carla.Rotation(pitch=-2.92, yaw=-81.08, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=16.07, y=201.49, z=1.62),
                carla.Rotation(pitch=4.48, yaw=-86.75, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=15.66, y=114.98, z=2.02),
                carla.Rotation(pitch=-4.25, yaw=-89.70, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=15.56, y=65.11, z=1.63),
                carla.Rotation(pitch=-8.32, yaw=-90.19, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=15.22, y=-5.02, z=1.72),
                carla.Rotation(pitch=-4.13, yaw=-92.21, roll=0.00),
            ),
            carla.Transform(
                carla.Location(x=15.13, y=-51.77, z=1.76),
                carla.Rotation(pitch=1.01, yaw=-93.02, roll=0.00),
            ),
        ]

        self.first_lane_tf_wps = [
            get_same_dir_lanes(self._map.get_waypoint(i.location))[3] for i in self.wp_transforms_tf
        ]
        self.second_lane_tf_wps = [
            get_same_dir_lanes(self._map.get_waypoint(i.location))[2] for i in self.wp_transforms_tf
        ]
        self.third_lane_tf_wps = [
            get_same_dir_lanes(self._map.get_waypoint(i.location))[1] for i in self.wp_transforms_tf
        ]
        self.fourth_lane_tf_wps = [
            get_same_dir_lanes(self._map.get_waypoint(i.location))[0] for i in self.wp_transforms_tf
        ]

        self.first_lane_tf_locations = [i.transform.location for i in self.first_lane_tf_wps]
        self.second_lane_tf_locations = [i.transform.location for i in self.second_lane_tf_wps]
        self.third_lane_tf_locations = [i.transform.location for i in self.third_lane_tf_wps]
        self.fourth_lane_tf_locations = [i.transform.location for i in self.fourth_lane_tf_wps]

        self.ego_end_last_stretch = carla.Transform(
            carla.Location(x=201.68, y=198.62, z=2.95),
            carla.Rotation(pitch=-9.78, yaw=163.47, roll=-0.00),
        )

        first_lane_min_dist = int(self.config.other_parameters["other_vehicles"]["first_lane_min_dist"])
        first_lane_max_dist = int(self.config.other_parameters["other_vehicles"]["first_lane_max_dist"])
        second_lane_min_dist = int(self.config.other_parameters["other_vehicles"]["second_lane_min_dist"])
        second_lane_max_dist = int(self.config.other_parameters["other_vehicles"]["second_lane_max_dist"])
        fourth_lane_min_dist = int(self.config.other_parameters["other_vehicles"]["fourth_lane_min_dist"])
        fourth_lane_max_dist = int(self.config.other_parameters["other_vehicles"]["fourth_lane_max_dist"])

        self._first_lane_distance_range = [first_lane_min_dist, first_lane_max_dist]
        self._second_lane_distance_range = [second_lane_min_dist, second_lane_max_dist]
        self._fourth_lane_distance_range = [fourth_lane_min_dist, fourth_lane_max_dist]

        self.sound_instructions_transformations_ff = {
            1: [
                "merge",
                carla.Transform(
                    carla.Location(x=394.56, y=57.48, z=0.00), carla.Rotation(pitch=0.00, yaw=-74.49, roll=0.00)
                ),
            ],
            2: [
                "stay_second",
                carla.Transform(
                    carla.Location(x=408.71, y=-34.60, z=0.00), carla.Rotation(pitch=360.00, yaw=270.44, roll=0.00)
                ),
            ],
            3: [
                "exit_bridge",
                carla.Transform(
                    carla.Location(x=145.67, y=-388.62, z=0.00), carla.Rotation(pitch=360.00, yaw=175.46, roll=0.00)
                ),
            ],
        }

        self.sound_instructions_transformations_sf = {
            1: [
                "merge",
                carla.Transform(
                    carla.Location(x=-124.30, y=79.44, z=7.21), carla.Rotation(pitch=364.08, yaw=282.20, roll=0.00)
                ),
            ],
            2: [
                "stay_second",
                carla.Transform(
                    carla.Location(x=-39.78, y=33.92, z=10.32), carla.Rotation(pitch=360.98, yaw=0.23, roll=0.00)
                ),
            ],
            3: [
                "exit",
                carla.Transform(
                    carla.Location(x=405.95, y=-252.05, z=0.00), carla.Rotation(pitch=360.00, yaw=262.90, roll=0.00)
                ),
            ],
        }

        self.sound_instructions_transformations_tf = {
            1: [
                "turn_right",
                carla.Transform(
                    carla.Location(x=-217.22, y=-99.58, z=0.00), carla.Rotation(pitch=0.00, yaw=162.58, roll=0.00)
                ),
            ],
            2: [
                "exit",
                carla.Transform(
                    carla.Location(x=-505.46, y=272.79, z=0.00), carla.Rotation(pitch=0.00, yaw=80.14, roll=0.00)
                ),
            ],
        }

        # print(actors)

        self.oncoming_traffic_light_location = carla.Location(x=-392.938782, y=0.607126, z=3.391296)
        self.ego_traffic_light_location = carla.Location(x=-389.239899, y=21.021969, z=6.675008)

        self.oncoming_traffic_light = self.find_traffic_light_closest_to_location(self.oncoming_traffic_light_location)
        self.ego_traffic_light = self.find_traffic_light_closest_to_location(self.ego_traffic_light_location)

        self.third_stretch_traffic_light_setter_transform = carla.Transform(
            carla.Location(x=-352.57, y=-68.13, z=0.00), carla.Rotation(pitch=0.00, yaw=-218.57, roll=0.00)
        )

        # world.player.set_light_state(self._lights)

        super(LeavingHighway, self).__init__(
            "LeavingHighway",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    def find_traffic_light_closest_to_location(self, location):
        closest_traffic_light = None
        min_distance = float("inf")

        traffic_lights = self.world.get_actors().filter("traffic.traffic_light")

        for traffic_light in traffic_lights:
            distance = traffic_light.get_transform().location.distance(location)
            if distance < min_distance:
                min_distance = distance
                closest_traffic_light = traffic_light

        return closest_traffic_light

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        self.set_traffic_light_times()

        weather = self.world.get_weather()

        if weather.sun_altitude_angle < 0.0:
            current_lights = carla.VehicleLightState.HighBeam
            current_lights ^= carla.VehicleLightState.Position
            self.ego_vehicles[0].set_light_state(carla.VehicleLightState(current_lights))

    def set_traffic_light_times(self, green_time=6, yellow_time=2, red_time=0):
        traffic_lights = self.world.get_actors().filter("traffic.traffic_light")

        for traffic_light in traffic_lights:
            traffic_light.set_green_time(green_time)
            traffic_light.set_yellow_time(yellow_time)
            traffic_light.set_red_time(red_time)

    def stop_constant_velocity(self):
        """Stops the constant velocity behavior"""
        self._is_constant_velocity_active = False
        for actor in self._actor_list:
            actor.disable_constant_velocity()
            self._tm.ignore_vehicles_percentage(actor, 0)

    def _create_behavior(self):
        first_stretch = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        traffic_flow_first_lane_ff = ActorFlowSections(
            self.first_lane_ff_wps[0],
            self.first_lane_ff_wps[-1],
            self._first_lane_distance_range,
            self.first_lane_locations,
            actor_speed=self.first_lane_speed,
            initial_velocity_vector=carla.Vector3D(self.first_lane_speed + 5, 0, -3),
        )
        traffic_flow_second_lane_ff = ActorFlowSections(
            self.second_lane_ff_wps[0],
            self.second_lane_ff_wps[-1],
            self._second_lane_distance_range,
            self.second_lane_locations,
            actor_speed=self.second_lane_speed,
            initial_velocity_vector=carla.Vector3D(self.second_lane_speed + 5, 0, -3),
        )
        traffic_flow_fourth_lane_ff = ActorFlowSections(
            self.fourth_lane_ff_wps[0],
            self.fourth_lane_ff_wps[-1],
            self._fourth_lane_distance_range,
            self.fourth_lane_locations,
            actor_speed=self.third_lane_speed,
            initial_velocity_vector=carla.Vector3D(self.third_lane_speed + 5, 0, -3),
        )

        first_stretch.add_child(traffic_flow_first_lane_ff)
        first_stretch.add_child(traffic_flow_second_lane_ff)
        first_stretch.add_child(traffic_flow_fourth_lane_ff)

        sound_instruction_sequence_ff = py_trees.composites.Sequence()

        for key, value in self.sound_instructions_transformations_ff.items():
            current_sequence = py_trees.composites.Sequence()
            current_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0], value[1].location, 5))

            current_sequence.add_child(
                PlayMp3File(
                    f"{self.assets_path}/{value[0]}.mp3",
                )
            )

            sound_instruction_sequence_ff.add_child(current_sequence)

        sound_instruction_sequence_ff.add_child(WaitForever())

        first_stretch.add_child(sound_instruction_sequence_ff)

        first_stretch.add_child(
            InTriggerDistanceToLocation(self.ego_vehicles[0], self.ego_end_first_stretch.location, distance=5)
        )

        second_stretch = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        traffic_flow_first_lane_sf = ActorFlowSections(
            self.first_lane_sf_wps[0],
            self.first_lane_sf_wps[-1],
            self._first_lane_distance_range,
            self.first_lane_sf_locations,
            actor_speed=self.first_lane_speed,
            initial_velocity_vector=carla.Vector3D(self.first_lane_speed + 5, 0, 0),
        )
        traffic_flow_second_lane_sf = ActorFlowSections(
            self.second_lane_sf_wps[0],
            self.second_lane_sf_wps[-1],
            self._second_lane_distance_range,
            self.second_lane_sf_locations,
            actor_speed=self.second_lane_speed,
            initial_velocity_vector=carla.Vector3D(self.second_lane_speed + 5, 0, 0),
        )
        traffic_flow_fourth_lane_sf = ActorFlowSections(
            self.fourth_lane_sf_wps[0],
            self.fourth_lane_sf_wps[-1],
            self._fourth_lane_distance_range,
            self.fourth_lane_sf_locations,
            actor_speed=self.third_lane_speed,
            initial_velocity_vector=carla.Vector3D(self.third_lane_speed + 5, 0, 0),
        )

        second_stretch.add_child(traffic_flow_first_lane_sf)
        second_stretch.add_child(traffic_flow_second_lane_sf)
        second_stretch.add_child(traffic_flow_fourth_lane_sf)

        sound_instruction_sequence_sf = py_trees.composites.Sequence()

        for key, value in self.sound_instructions_transformations_sf.items():
            current_sequence = py_trees.composites.Sequence()
            current_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0], value[1].location, 10))

            current_sequence.add_child(
                PlayMp3File(
                    f"{self.assets_path}/{value[0]}.mp3",
                )
            )

            sound_instruction_sequence_sf.add_child(current_sequence)

        sound_instruction_sequence_sf.add_child(WaitForever())

        second_stretch.add_child(sound_instruction_sequence_sf)

        second_stretch.add_child(
            InTriggerDistanceToLocation(self.ego_vehicles[0], self.ego_end_second_stretch.location, distance=10)
        )

        third_stretch = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        traffic_flow_first_lane_tf = ActorFlowSections(
            self.first_lane_tf_wps[0],
            self.first_lane_tf_wps[-1],
            self._first_lane_distance_range,
            self.first_lane_tf_locations,
            actor_speed=self.first_lane_speed,
            initial_velocity_vector=carla.Vector3D(self.first_lane_speed + 5, 0, -3),
        )
        traffic_flow_second_lane_tf = ActorFlowSections(
            self.second_lane_tf_wps[0],
            self.second_lane_tf_wps[-1],
            self._second_lane_distance_range,
            self.second_lane_tf_locations,
            actor_speed=self.second_lane_speed,
            initial_velocity_vector=carla.Vector3D(self.second_lane_speed + 5, 0, -3),
        )
        traffic_flow_fourth_lane_tf = ActorFlowSections(
            self.fourth_lane_tf_wps[0],
            self.fourth_lane_tf_wps[-1],
            self._fourth_lane_distance_range,
            self.fourth_lane_tf_locations,
            actor_speed=self.third_lane_speed,
            initial_velocity_vector=carla.Vector3D(self.third_lane_speed + 5, 0, -3),
        )

        third_stretch.add_child(traffic_flow_first_lane_tf)
        third_stretch.add_child(traffic_flow_second_lane_tf)
        third_stretch.add_child(traffic_flow_fourth_lane_tf)

        sound_instruction_sequence_tf = py_trees.composites.Sequence()

        for key, value in self.sound_instructions_transformations_tf.items():
            current_sequence = py_trees.composites.Sequence()
            current_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0], value[1].location, 10))

            current_sequence.add_child(
                PlayMp3File(
                    f"{self.assets_path}/{value[0]}.mp3",
                )
            )

            sound_instruction_sequence_tf.add_child(current_sequence)

        sound_instruction_sequence_tf.add_child(WaitForever())

        third_stretch.add_child(sound_instruction_sequence_tf)

        third_stretch.add_child(
            InTriggerDistanceToLocation(self.ego_vehicles[0], self.ego_end_last_stretch.location, distance=10)
        )

        # ego_traffic_light_setter = TrafficLightControllerSetter(self.ego_traffic_light, carla.TrafficLightState.Red, 10)
        # oncoming_traffic_light_setter = TrafficLightControllerSetter(
        #     self.oncoming_traffic_light, carla.TrafficLightState.Green, 10
        # )

        # third_stretch_traffic_light_branch = py_trees.composites.Parallel(
        #     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
        # )

        # third_stretch_traffic_light_branch.add_child(
        #     InTriggerDistanceToLocation(
        #         self.ego_vehicles[0], self.third_stretch_traffic_light_setter_transform.location, distance=15
        #     )
        # )
        # third_stretch_traffic_light_branch.add_child(ego_traffic_light_setter)
        # third_stretch_traffic_light_branch.add_child(oncoming_traffic_light_setter)
        # third_stretch_traffic_light_branch.add_child(WaitForever())

        # third_stretch.add_child(third_stretch_traffic_light_branch)
        # first_stretch.add_child(third_stretch_traffic_light_branch)

        root = py_trees.composites.Sequence()

        root.add_child(first_stretch)
        root.add_child(second_stretch)
        # root.add_child(third_stretch)

        return root

    def _create_test_criteria(self):
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        self.remove_all_actors()
