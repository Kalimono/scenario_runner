#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Introduction scenario:

A short introductory route with visual navigation markers and no events.
The ego vehicle follows gold waypoint markers through Town04. The scenario
ends when the ego reaches the final destination.
"""

import carla
import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import WaitForever
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance,
    InTriggerDistanceToLocation,
)
from srunner.scenariomanager.scenarioatomics.atomic_behaviors_custom import (
    KeepTrafficLightsGreen,
    NavigationMarker,
    SetEgoMaxSpeed,
)
from srunner.scenarios.basic_scenario import BasicScenario


# Visual navigation marker waypoints — gold dots shown sequentially.
_NAVIGATION_MARKERS = [
    carla.Transform(carla.Location(x=318.62, y=-72.53, z=0.00), carla.Rotation(pitch=360.00, yaw=225.16, roll=0.00)),      # 0
    carla.Transform(carla.Location(x=314.29, y=-146.37, z=0.00), carla.Rotation(pitch=360.00, yaw=270.51, roll=0.00)),     # 1
    carla.Transform(carla.Location(x=301.29, y=-172.44, z=0.20), carla.Rotation(pitch=360.00, yaw=180.33, roll=0.00)),     # 2
    carla.Transform(carla.Location(x=258.55, y=-210.65, z=0.02), carla.Rotation(pitch=0.00, yaw=-89.82, roll=0.00)),       # 3
    carla.Transform(carla.Location(x=258.82, y=-295.73, z=0.02), carla.Rotation(pitch=0.00, yaw=-89.82, roll=0.00)),       # 4
    carla.Transform(carla.Location(x=219.79, y=-311.29, z=0.00), carla.Rotation(pitch=360.00, yaw=180.59, roll=0.00)),     # 5
    carla.Transform(carla.Location(x=164.59, y=-311.25, z=0.03), carla.Rotation(pitch=0.00, yaw=-179.86, roll=0.00)),      # 6
]

_DEBUG_DRAW_NAV_MARKERS = True


class Introduction(BasicScenario):
    """
    Short introductory route with visual navigation markers and no events.
    The scenario ends when the ego vehicle reaches the final destination.
    """

    timeout = 9999999

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=9999999):
        self._world = world
        self.timeout = timeout
        self._end_location = carla.Location(x=58.69, y=-187.72, z=0.03)

        super(Introduction, self).__init__(
            "Introduction",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=False,
        )

        # Constant 40 km/h speed cap for the intro scenario.
        ego = self.ego_vehicles[0]
        if hasattr(ego, 'set_max_speed'):
            ego.set_max_speed(40.0)
            print("[Introduction] Ego max speed set to 40 km/h")

        root_tree = self.scenario_tree

        lights_branch = KeepTrafficLightsGreen()
        root_tree.add_child(lights_branch)
        lights_branch.setup(1)

        nav_markers_branch = self._create_navigation_markers_behavior()
        root_tree.add_child(nav_markers_branch)
        nav_markers_branch.setup(timeout=1)

        # Freeze all traffic lights to green.
        for tl in world.get_actors().filter("traffic.traffic_light*"):
            tl.set_state(carla.TrafficLightState.Green)
            tl.set_green_time(99999.0)
            tl.set_red_time(0.0)
            tl.set_yellow_time(0.0)
            tl.freeze(True)

        if _DEBUG_DRAW_NAV_MARKERS:
            _nav_color = carla.Color(255, 0, 255)
            for i, transform in enumerate(_NAVIGATION_MARKERS):
                loc = transform.location
                world.debug.draw_point(
                    carla.Location(x=loc.x, y=loc.y, z=loc.z + 1.5),
                    size=0.5,
                    color=_nav_color,
                    life_time=9999,
                )
                world.debug.draw_string(
                    carla.Location(x=loc.x, y=loc.y, z=loc.z + 3.0),
                    str(i),
                    draw_shadow=True,
                    color=_nav_color,
                    life_time=9999,
                    persistent_lines=True,
                )

        self.scenario_tree.setup(timeout=1)

    def _initialize_actors(self, config):
        pass

    def _create_behavior(self):
        seq = py_trees.composites.Sequence("Introduction")
        seq.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0],
                self._end_location,
                distance=5.0,
                name="ReachedDestination",
            )
        )
        return seq

    def _create_navigation_markers_behavior(self):
        seq = py_trees.composites.Sequence("NavigationMarkersBranch")
        seq.add_child(DriveDistance(self.ego_vehicles[0], 0.5, name="WaitForEgoToMove"))

        for i, transform in enumerate(_NAVIGATION_MARKERS):
            seq.add_child(NavigationMarker(
                self.ego_vehicles[0],
                transform.location,
                str(i),
                trigger_distance=15.0,
                name="NavMarker_{}".format(i),
            ))

        seq.add_child(WaitForever())
        return seq

    def _create_test_criteria(self):
        return []

    def __del__(self):
        self.remove_all_actors()
