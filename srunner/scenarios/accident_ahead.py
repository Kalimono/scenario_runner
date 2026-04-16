#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Accident Ahead scenario:

The ego vehicle drives a route through the map. Audio cues (PlayMp3) guide the
driver at each decision point. Partway along the route an accident scene with
two stationary vehicles and two ambulances blocks part of the highway. The
scenario ends when the ego vehicle reaches the final destination.
"""

import math

import carla
import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ActorDestroy, ActorFlowSections, Idle, WaitForever
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance,
    InTriggerDistanceToLocation,
)
from srunner.scenariomanager.scenarioatomics.atomic_behaviors_custom import (
    BikeNearMissEvent,
    DespawnBatch,
    FreezeActor,
    KeepTrafficLightsGreen,
    LogNavigationCue,
    NavigationMarker,
    PlayMp3,
    SetEgoMaxSpeed,
    SpawnActorGroup,
    StartWalkerControllers,
    StopFlowSpawning,
    WalkerWalkTo,
)

import sys as _sys, os as _os
_LSL_DIR = _os.path.join(_os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))), "LSL")
if _LSL_DIR not in _sys.path:
    _sys.path.insert(0, _LSL_DIR)
from ego_vehicle_stream import EgoVehicleLSLStream
from srunner.scenarios.basic_scenario import BasicScenario


# (location, label) pairs for audio navigation cues (commented out — replaced by visual markers).
# _NAVIGATION_CUES = [
#     (carla.Location(x=100.34, y=-170.09, z=0.20), "STRAIGHT"),
#     (carla.Location(x=175.20, y=-169.66, z=0.20), "LEFT"),
#     (carla.Location(x=202.55, y=-189.70, z=0.02), "RIGHT"),
#     (carla.Location(x=221.67, y=-245.98, z=0.00), "RIGHT"),
#     (carla.Location(x=255.09, y=-222.12, z=0.02), "LEFT"),
#     (carla.Location(x=275.43, y=-169.09, z=0.20), "STRAIGHT"),
#     (carla.Location(x=330.80, y=-168.77, z=0.20), "STRAIGHT"),
#     (carla.Location(x=299.49, y=13.42,   z=1.62), "ACCIDENT AHEAD"),
#     (carla.Location(x=7.71,   y=-184.95, z=0.00), "LEAVE HIGHWAY"),
#     (carla.Location(x=180.30, y=-364.47, z=0.00), "ENTERING RESIDENTIAL"),
#     (carla.Location(x=202.50, y=-340.14, z=0.02), "LEFT"),
#     (carla.Location(x=222.93, y=-307.76, z=0.00), "RIGHT"),
#     (carla.Location(x=255.30, y=-290.87, z=0.02), "LEFT"),
#     (carla.Location(x=279.70, y=-246.38, z=0.00), "RIGHT"),
# ]

# Visual navigation marker waypoints — gold dots shown sequentially.
_NAVIGATION_MARKERS = [
    carla.Transform(carla.Location(x=92.11, y=-282.11, z=0.03), carla.Rotation(pitch=0.00, yaw=-225.04, roll=0.00)),       # 0  (new)
    carla.Transform(carla.Location(x=76.35, y=-170.22, z=0.20), carla.Rotation(pitch=0.00, yaw=0.33, roll=0.00)),          # 1
    carla.Transform(carla.Location(x=165.10, y=-169.72, z=0.20), carla.Rotation(pitch=0.00, yaw=0.33, roll=0.00)),         # 2  (new)
    carla.Transform(carla.Location(x=202.37, y=-181.87, z=0.02), carla.Rotation(pitch=360.00, yaw=271.31, roll=0.00)),     # 3  (moved)
    carla.Transform(carla.Location(x=215.85, y=-245.94, z=0.00), carla.Rotation(pitch=360.00, yaw=359.61, roll=0.00)),     # 4  (moved)
    carla.Transform(carla.Location(x=255.12, y=-233.06, z=0.02), carla.Rotation(pitch=360.00, yaw=90.18, roll=0.00)),      # 5  (moved)
    carla.Transform(carla.Location(x=289.83, y=-169.01, z=0.20), carla.Rotation(pitch=0.00, yaw=0.33, roll=0.00)),         # 6
    carla.Transform(carla.Location(x=371.88, y=-168.54, z=0.20), carla.Rotation(pitch=0.00, yaw=-359.67, roll=0.00)),      # 7  (moved)
    carla.Transform(carla.Location(x=383.70, y=-21.43, z=0.00), carla.Rotation(pitch=0.00, yaw=98.52, roll=0.00)),         # 8
    carla.Transform(carla.Location(x=208.24, y=18.86, z=6.57), carla.Rotation(pitch=3.41, yaw=-179.02, roll=0.00)),        # 9
    carla.Transform(carla.Location(x=-100.74, y=16.31, z=9.28), carla.Rotation(pitch=-0.96, yaw=-539.92, roll=0.00)),      # 10
    carla.Transform(carla.Location(x=-401.73, y=16.30, z=0.00), carla.Rotation(pitch=0.00, yaw=179.63, roll=0.00)),        # 11
    carla.Transform(carla.Location(x=-503.59, y=155.85, z=0.00), carla.Rotation(pitch=0.00, yaw=90.36, roll=0.00)),        # 12
    carla.Transform(carla.Location(x=-303.24, y=425.45, z=0.00), carla.Rotation(pitch=0.00, yaw=-0.21, roll=0.00)),        # 13
    carla.Transform(carla.Location(x=-57.40, y=368.83, z=0.00), carla.Rotation(pitch=0.00, yaw=-43.57, roll=0.00)),        # 14
    carla.Transform(carla.Location(x=9.12, y=139.49, z=0.00), carla.Rotation(pitch=0.00, yaw=-450.29, roll=0.00)),         # 15
    carla.Transform(carla.Location(x=11.52, y=-107.38, z=0.00), carla.Rotation(pitch=0.00, yaw=-90.22, roll=0.00)),        # 16
    carla.Transform(carla.Location(x=24.87, y=-273.74, z=0.00), carla.Rotation(pitch=0.00, yaw=-68.24, roll=0.00)),        # 17
    carla.Transform(carla.Location(x=196.93, y=-364.34, z=0.00), carla.Rotation(pitch=0.00, yaw=0.43, roll=0.00)),         # 18
    carla.Transform(carla.Location(x=219.92, y=-307.79, z=0.00), carla.Rotation(pitch=0.00, yaw=0.59, roll=0.00)),         # 19 (moved)
    carla.Transform(carla.Location(x=255.29, y=-287.85, z=0.02), carla.Rotation(pitch=360.00, yaw=90.18, roll=0.00)),      # 20 (moved)
    carla.Transform(carla.Location(x=286.39, y=-246.42, z=0.00), carla.Rotation(pitch=360.00, yaw=359.61, roll=0.00)),     # 21
    carla.Transform(carla.Location(x=311.30, y=-203.93, z=0.00), carla.Rotation(pitch=0.00, yaw=90.51, roll=0.00)),        # 22
]

CUE_TRIGGER_DISTANCE = 20   # metres; wider tolerance so cues survive small route deviations
TRAFFIC_BATCH_SEED = 42     # fixed seed for reproducible autopilot behaviour
DESPAWN_DISTANCE   = 20     # metres from despawn point before actors are removed
_DEBUG_LAST_EVENT = False
_DEBUG_DRAW_MARKINGS = False
_DEBUG_DRAW_SOUNDCUES = False
_DEBUG_DRAW_NAV_MARKERS = True
_DEBUG_DRAW_BIKE_EVENT = False
EGO_MAX_SPEED_KMH = 40.0


class AccidentAhead(BasicScenario):
    """
    Ego vehicle drives a guided route past an accident scene.

    Other actors (two cars + two ambulances) are placed statically at the
    accident location on the highway. The scenario ends when the ego vehicle
    reaches the end transform.
    """

    timeout = 9999999

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=9999999):

        self._map = CarlaDataProvider.get_map()
        self._world = world
        self.timeout = timeout
        self._debug_trigger_transform = carla.Transform(
            carla.Location(x=233.73, y=-307.65, z=0.00),
            carla.Rotation(pitch=0.00, yaw=0.59, roll=0.00),
        )
        self._debug_bike_spawn_transform = carla.Transform(
            carla.Location(x=231.36, y=-293.92, z=0.17),
            carla.Rotation(pitch=0.00, yaw=-24.28, roll=0.00),
        )
        self._debug_sync_transform = carla.Transform(
            carla.Location(x=249.13, y=-302.82, z=0.16),
            carla.Rotation(pitch=0.00, yaw=-7.76, roll=0.00),
        )
        self._debug_bike_end_transform = carla.Transform(
            carla.Location(x=264.99, y=-304.01, z=0.16),
            carla.Rotation(pitch=0.00, yaw=-3.75, roll=0.00),
        )

        # Accident-scene actor transforms.
        # Pitch/roll kept from original capture so vehicles land in a natural crashed pose.
        # z raised by 3 m above capture point to avoid spawn collisions; gravity settles them.
        self._accident_actor_spawns = [
            ("vehicle.tesla.model3", carla.Transform(
                carla.Location(x=-390.85, y=6.40,  z=4.00),
                carla.Rotation(pitch=0.0, yaw=119.04,  roll=-0.01),
            )),
            ("vehicle.audi.a2", carla.Transform(
                carla.Location(x=-386.93, y=6.43,  z=3.00),
                carla.Rotation(pitch=0.0, yaw=178.91,  roll=-0.01),
            )),
            ("vehicle.ford.ambulance", carla.Transform(
                carla.Location(x=-379.08, y=10.53, z=3.00),
                carla.Rotation(pitch=0.0, yaw=-179.86, roll=-0.01),
            )),
            ("vehicle.ford.ambulance", carla.Transform(
                carla.Location(x=-405.22, y=7.52,  z=3.00),
                carla.Rotation(pitch=0.0, yaw=169.75,  roll=-0.01),
            )),
            ("vehicle.dodge.charger_police", carla.Transform(
                carla.Location(x=-377.03, y=2.44,  z=3.00),
                carla.Rotation(pitch=0.0, yaw=135.98, roll=0.00),
            )),
            ("vehicle.carlamotors.firetruck", carla.Transform(
                carla.Location(x=-385.94, y=-2.37, z=3.00),
                carla.Rotation(pitch=0.0, yaw=124.58, roll=0.00),
            )),
            # Construction site 1
            ("vehicle.mercedes.sprinter", carla.Transform(
                carla.Location(x=56.08, y=-175.15, z=1.00),
                carla.Rotation(pitch=0.00, yaw=-130.00, roll=0.00),
            )),
            ("vehicle.mercedes.sprinter", carla.Transform(
                carla.Location(x=200.01, y=-229.79, z=0.02),
                carla.Rotation(pitch=0.00, yaw=90.00, roll=0.00),
            )),
        ]

        # Police pedestrians at the accident scene — spawned at ground level, no z raise needed.
        self._pedestrian_spawns = [
            ("walker.pedestrian.0030", carla.Transform(
                carla.Location(x=-376.57, y=5.02,  z=-0.01),
                carla.Rotation(pitch=0.00, yaw=19.64,   roll=0.00),
            )),
            ("walker.pedestrian.0030", carla.Transform(
                carla.Location(x=-388.09, y=10.27, z=-0.00),
                carla.Rotation(pitch=0.00, yaw=-117.29, roll=0.00),
            )),
            ("walker.pedestrian.0032", carla.Transform(
                carla.Location(x=-397.67, y=7.48,  z=-0.00),
                carla.Rotation(pitch=0.00, yaw=-22.46,  roll=0.00),
            )),
            ("walker.pedestrian.0032", carla.Transform(
                carla.Location(x=-388.80, y=1.10,  z=-0.00),
                carla.Rotation(pitch=0.00, yaw=43.09,   roll=0.00),
            )),
        ]

        # ------------------------------------------------------------------
        # Batch 1 — ambient traffic, spawns at scenario start (trigger = ego spawn point).
        # Vehicles get autopilot; pedestrians get AI walker controllers.
        # All destroyed when the ego reaches the batch 1 despawn point.
        # ------------------------------------------------------------------
        self._batch1_vehicle_spawns = [
            ("vehicle.audi.tt", carla.Transform(
                carla.Location(x=62.25, y=-198.15, z=0.03),
                carla.Rotation(pitch=360.00, yaw=270.35, roll=0.00),
            )),
            # ("vehicle.chevrolet.impala", carla.Transform(
            #     carla.Location(x=187.81, y=-173.09, z=0.20),
            #     carla.Rotation(pitch=360.00, yaw=180.33, roll=0.00),
            # )),
            ("vehicle.nissan.patrol", carla.Transform(
                carla.Location(x=201.46, y=-294.96, z=0.02),
                carla.Rotation(pitch=0.00, yaw=91.31, roll=0.00),
            )),
        ]
        self._batch1_pedestrian_spawns = [
            ("walker.pedestrian.0001", carla.Transform(
                carla.Location(x=101.89, y=-297.99, z=0.03),
                carla.Rotation(pitch=0.00, yaw=-215.11, roll=0.00),
            )),
            ("walker.pedestrian.0002", carla.Transform(
                carla.Location(x=262.35, y=-181.47, z=0.02),
                carla.Rotation(pitch=0.00, yaw=-89.82, roll=0.00),
            )),
            ("walker.pedestrian.0003", carla.Transform(
                carla.Location(x=262.19, y=-131.14, z=0.02),
                carla.Rotation(pitch=0.00, yaw=-89.82, roll=0.00),
            )),
        ]
        self._batch1_despawn  = carla.Location(x=381.92, y=-162.32, z=0.00)
        self._batch1_actors   = []   # populated in _initialize_actors

        # ------------------------------------------------------------------
        # Traffic flows — ActorFlowSections replaces manual batch2 spawns.
        # Waypoints are resolved after super().__init__() below.
        # ------------------------------------------------------------------
        # Flow A (lanes -2 and -3): long highway flow, pre-populated.
        self._flow_source_loc = carla.Location(x=386.07, y=-225.10, z=0.00)
        self._flow_a_sink_loc = carla.Location(x=-434.32, y=6.58,   z=0.00)
        # Flow B (lane -4): short local flow, not pre-populated.
        self._flow_b_sink_loc = carla.Location(x=213.97,  y=-172.94, z=0.20)
        # Flow C — oncoming single lane from highway junction area.
        self._flow_c_source_loc = carla.Location(x=347.05, y=124.85,  z=0.00)
        self._flow_c_sink_loc   = carla.Location(x=273.24, y=-390.71, z=0.00)
        # Flow D — oncoming two-lane highway flow.
        self._flow_d_source_loc = carla.Location(x=-15.93, y=29.81,   z=0.00)
        self._flow_d_sink_loc   = carla.Location(x=273.24, y=-390.71, z=0.00)
        # Flow E — oncoming single lane, no initial speed.
        self._flow_e_source_loc = carla.Location(x=-56.63, y=144.25,  z=0.28)
        self._flow_e_sink_loc   = carla.Location(x=273.24, y=-390.71, z=0.00)
        # Flows A/B/C stop spawning (but don't delete existing actors) here.
        self._flow_abc_stop_loc = carla.Location(x=66.65, y=-335.76, z=0.00)
        # Flows D/E fully terminate here.
        self._flow_de_stop_loc  = carla.Location(x=-86.11, y=16.33,  z=9.53)

        # Flow F — accompanies A/B, short urban outbound loop.
        self._flow_f_source_loc = carla.Location(x=393.14, y=-233.83, z=0.00)
        self._flow_f_sink_loc   = carla.Location(x=267.62, y=-373.79, z=0.00)
        # Flow G — mid-route outbound feed.
        self._flow_g_source_loc = carla.Location(x=332.32, y=-64.13, z=0.00)
        self._flow_g_sink_loc   = carla.Location(x=-108.06, y=-77.40,  z=5.19)

        # ------------------------------------------------------------------
        # Batch 3 — urban area, spawned on demand when ego reaches trigger.
        # ------------------------------------------------------------------
        self._batch3_vehicle_spawns = [
            ("vehicle.audi.tt",          carla.Transform(carla.Location(x=205.57, y=-321.30, z=0.02), carla.Rotation(pitch=360.00, yaw=271.31, roll=0.00))),
            ("vehicle.chevrolet.impala", carla.Transform(carla.Location(x=215.34, y=-307.84, z=0.00), carla.Rotation(pitch=0.00, yaw=0.59, roll=0.00))),
        ]
        self._batch3_walker_spawns = [
            ("walker.pedestrian.0002", carla.Transform(carla.Location(x=208.17, y=-265.32, z=0.02), carla.Rotation(pitch=360.00, yaw=271.31, roll=0.00))),
            ("walker.pedestrian.0003", carla.Transform(carla.Location(x=300.70, y=-253.91, z=0.00), carla.Rotation(pitch=0.00, yaw=179.61, roll=0.00))),
        ]
        self._batch3_bicycle_spawns = []
        self._batch3_trigger  = carla.Location(x=201.65, y=-362.92, z=0.01)
        self._batch3_despawn  = carla.Location(x=349.03, y=-219.10, z=0.00)
        self._batch3_actors   = []

        # Construction site 1 — street barriers.
        self._construction_site1_barrier_spawns = [
            carla.Transform(carla.Location(x=49.55, y=-174.63, z=0.20), carla.Rotation(pitch=0.00, yaw=269.32,  roll=0.00)),
            carla.Transform(carla.Location(x=49.60, y=-170.51, z=0.20), carla.Rotation(pitch=0.00, yaw=-87.17,  roll=0.00)),
            carla.Transform(carla.Location(x=199.63, y=-217.18, z=0.02), carla.Rotation(pitch=0.00, yaw=180.47, roll=0.00)),
            carla.Transform(carla.Location(x=200.19, y=-238.21, z=0.02), carla.Rotation(pitch=0.00, yaw=-2.23,  roll=0.00)),
        ]

        # Construction site 1 — traffic cones lining the road.
        # Construction site 2 cones are appended below.
        self._construction_site1_cone_spawns = [
            carla.Transform(carla.Location(x=56.36, y=-179.00, z=0.06), carla.Rotation(pitch=0.00, yaw=51.04,  roll=0.00)),
            carla.Transform(carla.Location(x=57.26, y=-177.21, z=0.11), carla.Rotation(pitch=0.00, yaw=-31.43, roll=0.00)),
            carla.Transform(carla.Location(x=58.37, y=-175.55, z=0.13), carla.Rotation(pitch=0.00, yaw=-33.57, roll=0.00)),
            carla.Transform(carla.Location(x=59.41, y=-173.85, z=0.14), carla.Rotation(pitch=0.00, yaw=-31.40, roll=0.00)),
            carla.Transform(carla.Location(x=60.74, y=-172.64, z=0.15), carla.Rotation(pitch=0.00, yaw=-63.82, roll=0.00)),
            carla.Transform(carla.Location(x=62.54, y=-171.76, z=0.16), carla.Rotation(pitch=0.00, yaw=-64.03, roll=0.00)),
            carla.Transform(carla.Location(x=64.32, y=-170.85, z=0.17), carla.Rotation(pitch=0.00, yaw=-51.49, roll=0.00)),
            carla.Transform(carla.Location(x=65.89, y=-169.61, z=0.20), carla.Rotation(pitch=0.00, yaw=-51.77, roll=0.00)),
            # Construction site 2 cones
            carla.Transform(carla.Location(x=201.30, y=-235.25, z=0.02), carla.Rotation(pitch=0.00, yaw=4.39,   roll=0.00)),
            carla.Transform(carla.Location(x=200.99, y=-225.27, z=0.02), carla.Rotation(pitch=0.00, yaw=3.26,   roll=0.00)),
            carla.Transform(carla.Location(x=200.94, y=-223.27, z=0.02), carla.Rotation(pitch=0.00, yaw=1.58,   roll=0.00)),
            carla.Transform(carla.Location(x=200.72, y=-221.28, z=0.02), carla.Rotation(pitch=0.00, yaw=-1.98,  roll=0.00)),
            carla.Transform(carla.Location(x=200.82, y=-219.30, z=0.02), carla.Rotation(pitch=0.00, yaw=1.64,   roll=0.00)),
            carla.Transform(carla.Location(x=200.94, y=-217.30, z=0.02), carla.Rotation(pitch=0.00, yaw=-12.16, roll=0.00)),
        ]

        # Construction site 2 pedestrian event.
        self._cs2_ped_spawn = carla.Transform(
            carla.Location(x=199.55, y=-234.29, z=1.02),
            carla.Rotation(pitch=0.00, yaw=32.93, roll=0.00),
        )
        self._cs2_ped_trigger = carla.Location(x=203.48, y=-237.06, z=0.02)
        self._cs2_ped_target  = carla.Location(x=202.72, y=-232.22, z=1.95)
        self._cs2_ped_actor   = None

        # ------------------------------------------------------------------
        # Cognitive load variant — "High" in config.name → more traffic & props
        # ------------------------------------------------------------------
        self._high_load = "High" in config.name
        if self._high_load:
            # Extra autopilot traffic for high cognitive load.
            self._batch1_vehicle_spawns.extend([
                ("vehicle.lincoln.mkz_2020", carla.Transform(
                    carla.Location(x=62.18, y=-186.27, z=0.03),
                    carla.Rotation(pitch=360.00, yaw=270.35, roll=0.00))),
                ("vehicle.dodge.charger_2020", carla.Transform(
                    carla.Location(x=246.02, y=-122.52, z=0.02),
                    carla.Rotation(pitch=360.00, yaw=180.92, roll=0.00))),
            ])
            self._batch1_pedestrian_spawns.extend([
                ("walker.pedestrian.0005", carla.Transform(
                    carla.Location(x=97.54, y=-274.24, z=0.03),
                    carla.Rotation(pitch=360.00, yaw=-46.14, roll=0.00))),
                ("walker.pedestrian.0006", carla.Transform(
                    carla.Location(x=87.74, y=-286.05, z=0.03),
                    carla.Rotation(pitch=0.00, yaw=-225.20, roll=0.00))),
                ("walker.pedestrian.0007", carla.Transform(
                    carla.Location(x=59.35, y=-233.17, z=0.03),
                    carla.Rotation(pitch=0.00, yaw=-254.31, roll=0.00))),
                ("walker.pedestrian.0008", carla.Transform(
                    carla.Location(x=69.14, y=-215.19, z=0.03),
                    carla.Rotation(pitch=360.00, yaw=-82.21, roll=0.00))),
                ("walker.pedestrian.0009", carla.Transform(
                    carla.Location(x=101.75, y=-179.46, z=0.20),
                    carla.Rotation(pitch=360.00, yaw=180.33, roll=0.00))),
                ("walker.pedestrian.0010", carla.Transform(
                    carla.Location(x=223.75, y=-176.77, z=0.20),
                    carla.Rotation(pitch=360.00, yaw=180.33, roll=0.00))),
            ])
            self._batch3_walker_spawns.extend([
                ("walker.pedestrian.0011", carla.Transform(
                    carla.Location(x=195.63, y=-297.40, z=0.02),
                    carla.Rotation(pitch=0.00, yaw=91.31, roll=0.00))),
                ("walker.pedestrian.0012", carla.Transform(
                    carla.Location(x=195.54, y=-293.45, z=0.02),
                    carla.Rotation(pitch=0.00, yaw=91.31, roll=0.00))),
                ("walker.pedestrian.0013", carla.Transform(
                    carla.Location(x=195.44, y=-288.80, z=0.02),
                    carla.Rotation(pitch=0.00, yaw=91.31, roll=0.00))),
                ("walker.pedestrian.0014", carla.Transform(
                    carla.Location(x=262.59, y=-259.26, z=0.02),
                    carla.Rotation(pitch=0.00, yaw=-89.82, roll=0.00))),
                ("walker.pedestrian.0015", carla.Transform(
                    carla.Location(x=251.42, y=-292.80, z=0.02),
                    carla.Rotation(pitch=360.00, yaw=90.18, roll=0.00))),
            ])
            self._batch3_bicycle_spawns.extend([
                ("vehicle.gazelle.omafiets", carla.Transform(
                    carla.Location(x=201.52, y=-297.33, z=0.02),
                    carla.Rotation(pitch=0.00, yaw=91.31, roll=0.00))),
            ])
            # Extra static props — parked cars (spawned and frozen).
            self._extra_parked_spawns = []
            # Extra construction cones.
            self._extra_cone_spawns = [
                carla.Transform(carla.Location(x=67.50, y=-169.10, z=0.20), carla.Rotation(yaw=-51.77)),
                carla.Transform(carla.Location(x=69.30, y=-168.40, z=0.20), carla.Rotation(yaw=-51.77)),
                carla.Transform(carla.Location(x=71.10, y=-168.00, z=0.20), carla.Rotation(yaw=-51.77)),
                carla.Transform(carla.Location(x=73.00, y=-167.80, z=0.20), carla.Rotation(yaw=-51.77)),
            ]
            self._extra_static_pedestrian_spawns = [
                ("walker.pedestrian.0027", carla.Transform(carla.Location(x=207.20, y=-216.10, z=0.17), carla.Rotation(pitch=0.00, yaw=91.61, roll=0.00))),
                ("walker.pedestrian.0028", carla.Transform(carla.Location(x=207.09, y=-213.76, z=0.17), carla.Rotation(pitch=0.00, yaw=-87.13, roll=0.00))),
            ]
            self._batch3_static_pedestrian_spawns = [
                ("walker.pedestrian.0016", carla.Transform(carla.Location(x=197.54, y=-321.57, z=1.67), carla.Rotation(pitch=0.00, yaw=95.89, roll=0.00))),
                ("walker.pedestrian.0017", carla.Transform(carla.Location(x=197.15, y=-318.61, z=1.67), carla.Rotation(pitch=0.00, yaw=-84.78, roll=0.00))),
                ("walker.pedestrian.0018", carla.Transform(carla.Location(x=214.39, y=-300.45, z=1.67), carla.Rotation(pitch=0.00, yaw=-91.62, roll=0.00))),
                ("walker.pedestrian.0019", carla.Transform(carla.Location(x=217.13, y=-300.53, z=1.67), carla.Rotation(pitch=0.00, yaw=-91.62, roll=0.00))),
                ("walker.pedestrian.0020", carla.Transform(carla.Location(x=250.84, y=-297.52, z=1.67), carla.Rotation(pitch=0.00, yaw=-3.02, roll=0.00))),
                ("walker.pedestrian.0021", carla.Transform(carla.Location(x=251.15, y=-299.70, z=1.67), carla.Rotation(pitch=0.00, yaw=-4.61, roll=0.00))),
                ("walker.pedestrian.0022", carla.Transform(carla.Location(x=268.62, y=-298.12, z=1.67), carla.Rotation(pitch=0.00, yaw=92.98, roll=0.00))),
                ("walker.pedestrian.0023", carla.Transform(carla.Location(x=268.46, y=-292.34, z=1.67), carla.Rotation(pitch=0.00, yaw=-91.69, roll=0.00))),
                #("walker.pedestrian.0024", carla.Transform(carla.Location(x=244.45, y=-300.44, z=1.67), carla.Rotation(pitch=0.00, yaw=-116.72, roll=0.00))),
                #("walker.pedestrian.0025", carla.Transform(carla.Location(x=242.07, y=-299.60, z=1.67), carla.Rotation(pitch=0.00, yaw=-109.26, roll=0.00))),
                ("walker.pedestrian.0026", carla.Transform(carla.Location(x=238.72, y=-299.27, z=1.67), carla.Rotation(pitch=0.00, yaw=-97.30, roll=0.00))),
            ]
        else:
            # Low cognitive load — fewer actors.
            self._extra_parked_spawns = []
            self._extra_cone_spawns = []
            self._extra_static_pedestrian_spawns = []
            self._batch3_static_pedestrian_spawns = []

        # Scenario end location
        self._end_location = carla.Location(x=311.41, y=-216.00, z=0.00)

        self._batch1_walker_controllers = []

        super(AccidentAhead, self).__init__(
            "AccidentAhead",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=False,
        )
        self._set_ego_speed_cap(EGO_MAX_SPEED_KMH)

        # Fixed seed so pedestrian navigation destinations are reproducible.
        world.set_pedestrians_seed(TRAFFIC_BATCH_SEED)
        world.set_pedestrians_cross_factor(0.0)

        if _DEBUG_LAST_EVENT:
            return

        root_tree = self.scenario_tree

        # Speed zone branch — adjusts ego max speed at trigger locations.
        speed_zone_branch = self._create_speed_zone_behavior()
        root_tree.add_child(speed_zone_branch)
        speed_zone_branch.setup(timeout=1)

        # Construction site 2 pedestrian event branch.
        if self._cs2_ped_actor is not None:
            cs2_ped_branch = self._create_cs2_ped_behavior()
            root_tree.add_child(cs2_ped_branch)
            cs2_ped_branch.setup(timeout=1)

        lights_branch = KeepTrafficLightsGreen()
        root_tree.add_child(lights_branch)
        lights_branch.setup(1)

        # Ego vehicle telemetry → LSL stream (background thread, not in behavior tree).
        self._lsl_stream = EgoVehicleLSLStream(
            self.ego_vehicles[0],
            participant_id=getattr(config, 'participant_id', ''))
        self._lsl_stream.start()

        # Sound cue navigation — commented out, replaced by visual markers.
        # play_mp3_branch = self._create_play_mp3_behavior()
        # root_tree.add_child(play_mp3_branch)
        # play_mp3_branch.setup(timeout=1)

        # Visual navigation markers — gold dots shown one at a time.
        nav_markers_branch = self._create_navigation_markers_behavior()
        root_tree.add_child(nav_markers_branch)
        nav_markers_branch.setup(timeout=1)

        # Batch 1 despawn branch — destroys ambient traffic when ego passes the despawn point.
        batch1_branch = self._create_batch_despawn_behavior(
            self._batch1_actors, self._batch1_despawn, "Batch1Despawn"
        )
        root_tree.add_child(batch1_branch)
        batch1_branch.setup(timeout=1)

        if self._batch1_walker_controllers:
            batch1_walker_start_seq = py_trees.composites.Sequence("Batch1WalkerStartBranch")
            batch1_walker_start = StartWalkerControllers(
                world,
                self._batch1_walker_controllers,
                speed=1.4,
                destination_mode="far_nav",
                name="Batch1WalkerStart",
            )
            batch1_walker_start_seq.add_child(batch1_walker_start)
            batch1_walker_start_seq.add_child(WaitForever())
            root_tree.add_child(batch1_walker_start_seq)
            batch1_walker_start_seq.setup(timeout=1)

        # Batch 3 — spawn on trigger, despawn when ego exits the urban area.
        tm_port = CarlaDataProvider.get_traffic_manager_port()
        batch3_branch = self._create_batch3_behavior(tm_port)
        root_tree.add_child(batch3_branch)
        batch3_branch.setup(timeout=1)


        # Traffic flows — resolve waypoints by lane ID and add as parallel branches.
        source_wp  = self._map.get_waypoint(self._flow_source_loc, project_to_road=True,
                                             lane_type=carla.LaneType.Driving)
        a_sink_wp  = self._map.get_waypoint(self._flow_a_sink_loc, project_to_road=True,
                                             lane_type=carla.LaneType.Driving)
        b_sink_wp  = self._map.get_waypoint(self._flow_b_sink_loc, project_to_road=True,
                                             lane_type=carla.LaneType.Driving)

        source_l2 = self._waypoint_at_lane(source_wp, -2)
        source_l3 = self._waypoint_at_lane(source_wp, -3)
        source_l4 = self._waypoint_at_lane(source_wp, -4)
        a_sink_l2 = self._waypoint_at_lane(a_sink_wp, -2)
        a_sink_l3 = self._waypoint_at_lane(a_sink_wp, -3)
        b_sink_l4 = self._waypoint_at_lane(b_sink_wp, -4)


        _FLOWS_ENABLED = True
        _ONCOMING_FLOWS_ENABLED = False

        if _FLOWS_ENABLED and source_l2 and a_sink_l2:
            flow_a2 = ActorFlowSections(
                source_l2, a_sink_l2,
                spawn_dist_interval=(360, 420),
                sections=[source_l2.transform.location, a_sink_l2.transform.location],
                actor_speed=75 / 3.6,
                initial_actors=True,
                allow_lane_change=True,
                name="FlowA_Lane-2",
            )
            branch_a2 = self._flow_branch_stop_spawn(flow_a2, self._flow_abc_stop_loc)
            root_tree.add_child(branch_a2)
            branch_a2.setup(timeout=1)

        if _FLOWS_ENABLED and source_l3 and a_sink_l3:
            flow_a3 = ActorFlowSections(
                source_l3, a_sink_l3,
                spawn_dist_interval=(360, 420),
                sections=[source_l3.transform.location, a_sink_l3.transform.location],
                actor_speed=75 / 3.6,
                initial_actors=True,
                allow_lane_change=True,
                name="FlowA_Lane-3",
            )
            branch_a3 = self._flow_branch_stop_spawn(flow_a3, self._flow_abc_stop_loc)
            root_tree.add_child(branch_a3)
            branch_a3.setup(timeout=1)

        if _FLOWS_ENABLED and source_l4 and b_sink_l4:
            flow_b4 = ActorFlowSections(
                source_l4, b_sink_l4,
                spawn_dist_interval=(360, 420),
                sections=[source_l4.transform.location, b_sink_l4.transform.location],
                actor_speed=40 / 3.6,
                initial_actors=False,
                allow_lane_change=True,
                name="FlowB_Lane-4",
            )
            branch_b4 = self._flow_branch_stop_spawn(flow_b4, self._flow_abc_stop_loc)
            root_tree.add_child(branch_b4)
            branch_b4.setup(timeout=1)

        # Flow C — oncoming single lane
        c_source_wp = self._map.get_waypoint(self._flow_c_source_loc, project_to_road=True,
                                              lane_type=carla.LaneType.Driving)
        c_sink_wp   = self._map.get_waypoint(self._flow_c_sink_loc,   project_to_road=True,
                                              lane_type=carla.LaneType.Driving)
        if _ONCOMING_FLOWS_ENABLED and c_source_wp and c_sink_wp:
            flow_c = ActorFlowSections(
                c_source_wp, c_sink_wp,
                spawn_dist_interval=(520, 760),
                sections=[c_source_wp.transform.location, c_sink_wp.transform.location],
                actor_speed=75 / 3.6,
                initial_actors=False,
                allow_lane_change=True,
                name="FlowC_Oncoming1",
            )
            branch_c = self._flow_branch_stop_spawn(flow_c, self._flow_abc_stop_loc)
            root_tree.add_child(branch_c)
            branch_c.setup(timeout=1)

        # Flow D — oncoming two lanes (full terminate at de_stop_loc)
        d_source_wp  = self._map.get_waypoint(self._flow_d_source_loc, project_to_road=True,
                                               lane_type=carla.LaneType.Driving)
        d_sink_wp    = self._map.get_waypoint(self._flow_d_sink_loc,   project_to_road=True,
                                               lane_type=carla.LaneType.Driving)
        d_source_wp2 = d_source_wp.get_right_lane() if d_source_wp else None
        if d_source_wp2 is None or d_source_wp2.lane_type != carla.LaneType.Driving:
            d_source_wp2 = d_source_wp.get_left_lane() if d_source_wp else None
        if _ONCOMING_FLOWS_ENABLED and d_source_wp and d_sink_wp:
            flow_d1 = ActorFlowSections(
                d_source_wp, d_sink_wp,
                spawn_dist_interval=(520, 760),
                sections=[d_source_wp.transform.location, d_sink_wp.transform.location],
                actor_speed=75 / 3.6,
                initial_actors=False,
                name="FlowD_Oncoming2a",
            )
            branch_d1 = self._flow_branch(flow_d1, self._flow_de_stop_loc)
            root_tree.add_child(branch_d1)
            branch_d1.setup(timeout=1)
        if _ONCOMING_FLOWS_ENABLED and d_source_wp2 and d_sink_wp:
            flow_d2 = ActorFlowSections(
                d_source_wp2, d_sink_wp,
                spawn_dist_interval=(520, 760),
                sections=[d_source_wp2.transform.location, d_sink_wp.transform.location],
                actor_speed=75 / 3.6,
                initial_actors=False,
                name="FlowD_Oncoming2b",
            )
            branch_d2 = self._flow_branch(flow_d2, self._flow_de_stop_loc)
            root_tree.add_child(branch_d2)
            branch_d2.setup(timeout=1)

        # Flow E — oncoming single lane (full terminate at de_stop_loc)
        e_source_wp = self._map.get_waypoint(self._flow_e_source_loc, project_to_road=True,
                                              lane_type=carla.LaneType.Driving)
        e_sink_wp   = self._map.get_waypoint(self._flow_e_sink_loc,   project_to_road=True,
                                              lane_type=carla.LaneType.Driving)
        if _ONCOMING_FLOWS_ENABLED and e_source_wp and e_sink_wp:
            flow_e = ActorFlowSections(
                e_source_wp, e_sink_wp,
                spawn_dist_interval=(520, 760),
                sections=[e_source_wp.transform.location, e_sink_wp.transform.location],
                actor_speed=75 / 3.6,
                initial_actors=False,
                name="FlowE_Oncoming3",
            )
            branch_e = self._flow_branch(flow_e, self._flow_de_stop_loc)
            root_tree.add_child(branch_e)
            branch_e.setup(timeout=1)

        # Flow F — short outbound urban loop, accompanies A/B.
        f_source_wp = self._map.get_waypoint(self._flow_f_source_loc, project_to_road=True,
                                              lane_type=carla.LaneType.Driving)
        f_sink_wp   = self._map.get_waypoint(self._flow_f_sink_loc,   project_to_road=True,
                                              lane_type=carla.LaneType.Driving)
        if f_source_wp and f_sink_wp:
            flow_f = ActorFlowSections(
                f_source_wp, f_sink_wp,
                spawn_dist_interval=(360, 420),
                sections=[f_source_wp.transform.location, f_sink_wp.transform.location],
                actor_speed=110 / 3.6,
                initial_actors=True,
                allow_lane_change=True,
                name="FlowF_Outbound",
            )
            branch_f = self._flow_branch_stop_spawn(flow_f, self._flow_abc_stop_loc)
            root_tree.add_child(branch_f)
            branch_f.setup(timeout=1)

        # Flow G — mid-route outbound feed, accompanies A/B.
        g_source_wp = self._map.get_waypoint(self._flow_g_source_loc, project_to_road=True,
                                              lane_type=carla.LaneType.Driving)
        g_sink_wp   = self._map.get_waypoint(self._flow_g_sink_loc,   project_to_road=True,
                                              lane_type=carla.LaneType.Driving)
        if False and g_source_wp and g_sink_wp:  # Flow G source blocked — disabled
            flow_g = ActorFlowSections(
                g_source_wp, g_sink_wp,
                spawn_dist_interval=(140, 180),
                sections=[g_source_wp.transform.location, g_sink_wp.transform.location],
                actor_speed=50 / 3.6,
                initial_actors=False,
                allow_lane_change=True,
                name="FlowG_Outbound",
            )
            branch_g = self._flow_branch_stop_spawn(flow_g, self._flow_abc_stop_loc)
            root_tree.add_child(branch_g)
            branch_g.setup(timeout=1)

        if _DEBUG_DRAW_MARKINGS:
            # Draw flow source labels on the map for orientation.
            _label_color = carla.Color(0, 200, 255)
            _flow_labels = [
                (self._flow_source_loc,    "A/B"),
                (self._flow_c_source_loc,  "C"),
                (self._flow_d_source_loc,  "D"),
                (self._flow_e_source_loc,  "E"),
                (self._flow_f_source_loc,  "F"),
                (self._flow_g_source_loc,  "G"),
            ]
            for loc, letter in _flow_labels:
                label_loc = carla.Location(x=loc.x, y=loc.y, z=loc.z + 5.0)
                world.debug.draw_string(
                    label_loc,
                    letter,
                    draw_shadow=True,
                    color=_label_color,
                    life_time=9999,
                    persistent_lines=True,
                )

                world.debug.draw_point(
                    label_loc,
                    size=0.3,
                    color=_label_color,
                    life_time=9999,
                )

            # Marker on the ground at flow F sink so it's easy to see where vehicles despawn.
            # Marker at ego-triggered flow stop point.
            _stop_color = carla.Color(255, 0, 255)
            _stop = self._flow_abc_stop_loc
            for _dz in [0.5, 2.0, 4.0, 6.0, 8.0]:
                world.debug.draw_point(
                    carla.Location(x=_stop.x, y=_stop.y, z=_stop.z + _dz),
                    size=0.5,
                    color=_stop_color,
                    life_time=9999,
                )
            world.debug.draw_string(
                carla.Location(x=_stop.x, y=_stop.y, z=_stop.z + 10.0),
                "FLOW STOP",
                draw_shadow=True,
                color=_stop_color,
                life_time=9999,
            )

            _sink_color = carla.Color(255, 0, 255)
            _fsink = self._flow_f_sink_loc
            for _dz in [0.5, 2.0, 4.0, 6.0, 8.0]:
                world.debug.draw_point(
                    carla.Location(x=_fsink.x, y=_fsink.y, z=_fsink.z + _dz),
                    size=0.5,
                    color=_sink_color,
                    life_time=9999,
                )
            world.debug.draw_string(
                carla.Location(x=_fsink.x, y=_fsink.y, z=_fsink.z + 10.0),
                "F-SINK",
                draw_shadow=True,
                color=_sink_color,
                life_time=9999,
            )

            # Draw batch 3 pedestrian spawn points so they're visible in the editor.
            _ped_color = carla.Color(255, 200, 0)
            for i, (_, t) in enumerate(self._batch3_walker_spawns):
                _pl = t.location
                world.debug.draw_point(
                    carla.Location(x=_pl.x, y=_pl.y, z=_pl.z + 1.0),
                    size=0.2,
                    color=_ped_color,
                    life_time=9999,
                )
                world.debug.draw_string(
                    carla.Location(x=_pl.x, y=_pl.y, z=_pl.z + 2.5),
                    "P{}".format(i + 1),
                    draw_shadow=True,
                    color=_ped_color,
                    life_time=9999,
                    persistent_lines=True,
                )

            # Draw high-load static pedestrian markers for quick coordinate review.
            _static_ped_color = carla.Color(255, 80, 80)
            for i, (_, t) in enumerate(self._extra_static_pedestrian_spawns):
                _spl = t.location
                world.debug.draw_point(
                    carla.Location(x=_spl.x, y=_spl.y, z=_spl.z + 0.5),
                    size=0.25,
                    color=_static_ped_color,
                    life_time=9999,
                )
                world.debug.draw_string(
                    carla.Location(x=_spl.x, y=_spl.y, z=_spl.z + 2.5),
                    "SP{}".format(i + 1),
                    draw_shadow=True,
                    color=_static_ped_color,
                    life_time=9999,
                    persistent_lines=True,
                )

            _parked_color = carla.Color(80, 200, 255)
            for i, (_, t) in enumerate(self._extra_parked_spawns):
                _pl = t.location
                world.debug.draw_point(
                    carla.Location(x=_pl.x, y=_pl.y, z=_pl.z + 1.0),
                    size=0.25,
                    color=_parked_color,
                    life_time=9999,
                )
                world.debug.draw_string(
                    carla.Location(x=_pl.x, y=_pl.y, z=_pl.z + 2.5),
                    "PK{}".format(i + 1),
                    draw_shadow=True,
                    color=_parked_color,
                    life_time=9999,
                    persistent_lines=True,
                )

            _cone_color = carla.Color(255, 140, 0)
            for i, t in enumerate(self._extra_cone_spawns):
                _cl = t.location
                world.debug.draw_point(
                    carla.Location(x=_cl.x, y=_cl.y, z=_cl.z + 0.5),
                    size=0.18,
                    color=_cone_color,
                    life_time=9999,
                )
                world.debug.draw_string(
                    carla.Location(x=_cl.x, y=_cl.y, z=_cl.z + 2.0),
                    "CN{}".format(i + 1),
                    draw_shadow=True,
                    color=_cone_color,
                    life_time=9999,
                    persistent_lines=True,
                )

        if _DEBUG_DRAW_SOUNDCUES:
            # Draw navigation cue markers on the map.
            # Each cue gets a point + two labels: the instruction and the audio filename.
            _cue_color = carla.Color(0, 255, 128)
            for i, (location, label) in enumerate(_NAVIGATION_CUES):
                filename = label.replace(" ", "_") + ".wav"
                base = carla.Location(x=location.x, y=location.y, z=location.z)
                world.debug.draw_point(
                    carla.Location(x=base.x, y=base.y, z=base.z + 1.0),
                    size=0.3,
                    color=_cue_color,
                    life_time=9999,
                )
                world.debug.draw_string(
                    carla.Location(x=base.x, y=base.y, z=base.z + 4.0),
                    "[{}] {}".format(i, label),
                    draw_shadow=True,
                    color=_cue_color,
                    life_time=9999,
                    persistent_lines=True,
                )
                world.debug.draw_string(
                    carla.Location(x=base.x, y=base.y, z=base.z + 2.5),
                    filename,
                    draw_shadow=True,
                    color=carla.Color(180, 255, 180),
                    life_time=9999,
                    persistent_lines=True,
                )

        if _DEBUG_DRAW_NAV_MARKERS:
            _nav_color = carla.Color(255, 0, 255)  # screaming pink
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

        if _DEBUG_DRAW_BIKE_EVENT:
            _bike_spawn_color = carla.Color(0, 180, 255)
            _bike_trigger_color = carla.Color(255, 80, 80)
            _bike_sync_color = carla.Color(255, 210, 0)
            _bike_end_color = carla.Color(120, 255, 120)

            _bike_markers = [
                (self._debug_trigger_transform.location, "AA BIKE TRIGGER", _bike_trigger_color),
                (self._debug_bike_spawn_transform.location, "AA BIKE SPAWN", _bike_spawn_color),
                (self._debug_sync_transform.location, "AA BIKE SYNC", _bike_sync_color),
                (self._debug_bike_end_transform.location, "AA BIKE END", _bike_end_color),
            ]

            for location, label, color in _bike_markers:
                for dz in [0.5, 1.5, 3.0]:
                    world.debug.draw_point(
                        carla.Location(x=location.x, y=location.y, z=location.z + dz),
                        size=0.25,
                        color=color,
                        life_time=9999,
                    )
                world.debug.draw_string(
                    carla.Location(x=location.x, y=location.y, z=location.z + 4.5),
                    label,
                    draw_shadow=True,
                    color=color,
                    life_time=9999,
                    persistent_lines=True,
                )

        # Freeze all traffic lights to green.
        for tl in world.get_actors().filter("traffic.traffic_light*"):
            tl.set_state(carla.TrafficLightState.Green)
            tl.set_green_time(99999.0)
            tl.set_red_time(0.0)
            tl.set_yellow_time(0.0)
            tl.freeze(True)

        # BasicScenario sets up the tree before these extra branches are attached.
        # Re-setup once at the end so the full augmented tree is definitely live.
        self.scenario_tree.setup(timeout=1)

        # Teleport spectator to overlook the construction site 2 sprinter.
        try:
            spectator = world.get_spectator()
            spectator.set_transform(carla.Transform(
                carla.Location(x=200.01, y=-229.79, z=10.00),
                carla.Rotation(pitch=-45.00, yaw=-87.81, roll=0.00),
            ))
        except RuntimeError:
            pass

    # ------------------------------------------------------------------
    # Actor initialisation
    # ------------------------------------------------------------------

    def _initialize_actors(self, config):
        """Spawn accident-scene vehicles slightly above ground and let gravity settle them."""
        if _DEBUG_LAST_EVENT:
            return
        _EMERGENCY_LIGHTS = carla.VehicleLightState(
            carla.VehicleLightState.Special1 | carla.VehicleLightState.Special2
        )

        for blueprint, transform in self._accident_actor_spawns:
            actor = CarlaDataProvider.request_new_actor(blueprint, transform)
            if actor is not None:
                actor.set_simulate_physics(enabled=True)
                if "ambulance" in blueprint or "police" in blueprint or "firetruck" in blueprint:
                    actor.set_light_state(_EMERGENCY_LIGHTS)
                self.other_actors.append(actor)

        for blueprint, transform in self._pedestrian_spawns:
            actor = CarlaDataProvider.request_new_actor(blueprint, transform)
            if actor is not None:
                self.other_actors.append(actor)

        # Batch 1 ambient traffic — vehicles on autopilot, pedestrians get AI controllers.
        tm_port = CarlaDataProvider.get_traffic_manager_port()
        tm = CarlaDataProvider.get_client().get_trafficmanager(tm_port)
        tm.set_random_device_seed(TRAFFIC_BATCH_SEED)

        for blueprint, transform in self._batch1_vehicle_spawns:
            actor = CarlaDataProvider.request_new_actor(
                blueprint,
                transform,
                rolename='autopilot',
            )
            if actor is not None:
                actor.set_autopilot(True, tm_port)
                self._batch1_actors.append(actor)

        controller_bp = self._world.get_blueprint_library().find('controller.ai.walker')
        for blueprint, transform in self._batch1_pedestrian_spawns:
            walker = CarlaDataProvider.request_new_actor(blueprint, transform)
            if walker is not None:
                self._batch1_actors.append(walker)
                controller = self._world.try_spawn_actor(controller_bp, carla.Transform(), walker)
                if controller is not None:
                    self._batch1_actors.append(controller)
                    self._batch1_walker_controllers.append(controller)

        # Construction site 2 pedestrian.
        actor = CarlaDataProvider.request_new_actor("walker.pedestrian.0016", self._cs2_ped_spawn)
        if actor is not None:
            self._cs2_ped_actor = actor
            self.other_actors.append(actor)

        # Construction site barriers — spawned as static props via world directly.
        barrier_bp = self._world.get_blueprint_library().find("static.prop.streetbarrier")
        for transform in self._construction_site1_barrier_spawns:
            actor = self._world.try_spawn_actor(barrier_bp, transform)
            if actor is not None:
                self.other_actors.append(actor)

        # Construction site 1 cones — spawned as static props via world directly.
        cone_bp = self._world.get_blueprint_library().find("static.prop.constructioncone")
        for transform in self._construction_site1_cone_spawns:
            actor = self._world.try_spawn_actor(cone_bp, transform)
            if actor is not None:
                self.other_actors.append(actor)

        # High cognitive load extras — parked cars (frozen) and extra cones.
        for blueprint, transform in self._extra_parked_spawns:
            actor = CarlaDataProvider.request_new_actor(blueprint, transform)
            if actor is not None:
                actor.set_simulate_physics(enabled=False)
                self.other_actors.append(actor)
        for transform in self._extra_cone_spawns:
            actor = self._world.try_spawn_actor(cone_bp, transform)
            if actor is not None:
                self.other_actors.append(actor)
        for blueprint, transform in self._extra_static_pedestrian_spawns:
            actor = CarlaDataProvider.request_new_actor(blueprint, transform)
            if actor is not None:
                self.other_actors.append(actor)

    # ------------------------------------------------------------------
    # Main behavior: end when ego reaches destination
    # ------------------------------------------------------------------

    def _create_behavior(self):
        """
        Sequence:
          1. Freeze accident-scene actors after they settle (3 s).
          2. Wait for ego to pass the last batch despawn point (currently batch 1).
          3. Wait for ego to reach the final destination.
          4. Clean up accident-scene actors.
        Adding more batches later: insert their despawn location trigger before step 3.
        """
        if _DEBUG_LAST_EVENT:
            seq = py_trees.composites.Sequence("AccidentAheadDebugLastEvent")
            seq.add_child(
                InTriggerDistanceToLocation(
                    self.ego_vehicles[0],
                    self._debug_trigger_transform.location,
                    5.0,
                    name="DebugBikeTrigger",
                )
            )
            seq.add_child(
                BikeNearMissEvent(
                    self.ego_vehicles[0],
                    self._debug_bike_spawn_transform,
                    self._debug_sync_transform,
                    self._debug_bike_end_transform,
                    path_transforms=[
                        self._debug_sync_transform,
                        self._debug_bike_end_transform,
                    ],
                    blueprint="vehicle.diamondback.century",
                    min_speed=7.0,
                    max_speed=7.0,
                    fixed_speed=7.0,
                    name="DebugBikeNearMiss",
                )
            )
            seq.add_child(
                InTriggerDistanceToLocation(
                    self.ego_vehicles[0],
                    self._end_location,
                    distance=5.0,
                    name="DebugReachedDestination",
                )
            )
            return seq

        # Wait 3 s for accident vehicles to fall and settle, then freeze them in place.
        freeze_seq = py_trees.composites.Sequence("FreezeAccidentScene")
        freeze_seq.add_child(Idle(3.0, name="WaitForSettling"))
        for actor in self.other_actors:
            if actor.type_id.startswith("vehicle."):
                freeze_seq.add_child(FreezeActor(actor, name="FreezeActor_{}".format(actor.id)))

        end_condition = InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            self._end_location,
            distance=5.0,
            name="ReachedDestination",
        )

        root = py_trees.composites.Sequence("AccidentAhead")
        root.add_child(freeze_seq)
        root.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0],
                self._debug_trigger_transform.location,
                5.0,
                name="BikeNearMissTrigger",
            )
        )
        root.add_child(
            BikeNearMissEvent(
                self.ego_vehicles[0],
                self._debug_bike_spawn_transform,
                self._debug_sync_transform,
                self._debug_bike_end_transform,
                path_transforms=[
                    self._debug_sync_transform,
                    self._debug_bike_end_transform,
                ],
                blueprint="vehicle.diamondback.century",
                min_speed=7.0,
                max_speed=7.0,
                fixed_speed=7.0,
                name="BikeNearMiss",
            )
        )
        root.add_child(end_condition)

        # Clean up accident-scene actors on completion
        for actor in self.other_actors:
            root.add_child(ActorDestroy(actor))

        return root

    # ------------------------------------------------------------------
    # Batch traffic despawn branch (own root-level branch)
    # ------------------------------------------------------------------


    def _flow_branch_stop_spawn(self, flow, stop_location):
        """
        Keeps the flow running forever (existing actors reach the sink naturally)
        but stops new actors spawning once the ego reaches *stop_location*.
        Structure:
          Parallel(RUNNING forever):
            flow                         — always RUNNING
            Sequence: [TriggerDist → StopFlowSpawning(flow) → WaitForever]
        """
        par = py_trees.composites.Parallel(
            "{}_Branch".format(flow.name),
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
        )
        par.add_child(flow)
        stop_seq = py_trees.composites.Sequence("{}_StopSpawnSeq".format(flow.name))
        stop_seq.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0], stop_location,
                distance=DESPAWN_DISTANCE,
                name="{}_StopTrigger".format(flow.name),
            )
        )
        stop_seq.add_child(StopFlowSpawning(
            flow,
            ego_actor=self.ego_vehicles[0],
            stop_location=stop_location,
            name="{}_StopSpawn".format(flow.name),
        ))
        stop_seq.add_child(WaitForever())
        par.add_child(stop_seq)
        return par

    def _flow_branch(self, flow, stop_location=None):
        """
        Wraps *flow* in a Sequence so it never returns SUCCESS to the root Parallel:
          1. Parallel(SUCCESS_ON_ONE) — flow + optional stop trigger.
          2. WaitForever() — keeps the branch RUNNING after the flow stops.
        """
        seq = py_trees.composites.Sequence("{}_Branch".format(flow.name))

        if stop_location is not None:
            par = py_trees.composites.Parallel(
                "{}_StopParallel".format(flow.name),
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            )
            par.add_child(flow)
            par.add_child(
                InTriggerDistanceToLocation(
                    self.ego_vehicles[0], stop_location,
                    distance=DESPAWN_DISTANCE,
                    name="{}_StopTrigger".format(flow.name),
                )
            )
            seq.add_child(par)
        else:
            seq.add_child(flow)

        seq.add_child(WaitForever())
        return seq

    @staticmethod
    def _waypoint_at_lane(base_wp, target_lane_id):
        """
        Starting from *base_wp*, traverse left/right until the waypoint with
        *target_lane_id* is reached.  Returns None if the lane does not exist.
        """
        wp = base_wp
        for _ in range(10):
            if wp.lane_id == target_lane_id:
                return wp
            next_wp = wp.get_right_lane() if wp.lane_id > target_lane_id else wp.get_left_lane()
            if next_wp is None:
                return None
            wp = next_wp
        return None

    def _get_lane_end_location(self, actor_transform):
        """
        Return a destination at the end of the actor's current lane.

        We choose the lane end that best matches the actor yaw so walkers keep
        moving forward along their current sidewalk/road edge instead of
        constantly snapping to random navmesh targets.
        """
        lane_types = [
            carla.LaneType.Sidewalk,
            carla.LaneType.Shoulder,
            carla.LaneType.Biking,
            carla.LaneType.Driving,
            carla.LaneType.Any,
        ]

        waypoint = None
        for lane_type in lane_types:
            try:
                waypoint = self._map.get_waypoint(
                    actor_transform.location,
                    project_to_road=True,
                    lane_type=lane_type,
                )
            except RuntimeError:
                waypoint = None
            if waypoint is not None:
                break

        if waypoint is None:
            return self._world.get_random_location_from_navigation()

        next_wps = waypoint.next_until_lane_end(1.0)
        prev_wps = waypoint.previous_until_lane_start(1.0)

        next_loc = next_wps[-1].transform.location if next_wps else waypoint.transform.location
        prev_loc = prev_wps[-1].transform.location if prev_wps else waypoint.transform.location

        yaw = math.radians(actor_transform.rotation.yaw)
        actor_dir = carla.Vector3D(x=math.cos(yaw), y=math.sin(yaw), z=0.0)
        lane_dir = waypoint.transform.get_forward_vector()
        dot = actor_dir.x * lane_dir.x + actor_dir.y * lane_dir.y

        return next_loc if dot >= 0.0 else prev_loc

    def _create_batch3_behavior(self, tm_port):
        """
        Sequence:
          1. Wait for ego to reach the batch 3 spawn trigger.
          2. SpawnActorGroup — spawns cars, walkers, bicycles on demand.
          2b. Spawn static pedestrians (no AI controller).
          3. Wait for ego to reach the batch 3 despawn point.
          4. DespawnBatch — destroys all batch 3 actors.
          5. WaitForever so this branch never causes an early scenario end.
        """
        from srunner.scenariomanager.scenarioatomics.atomic_behaviors_custom import DespawnBatch
        from srunner.scenariomanager.scenarioatomics.atomic_behaviors import AtomicBehavior

        _static_spawns = self._batch3_static_pedestrian_spawns
        _actors_out = self._batch3_actors

        class _SpawnStaticPedestrians(AtomicBehavior):
            """Spawn static pedestrians (no walker controller)."""
            def update(self_inner):
                for blueprint, transform in _static_spawns:
                    actor = CarlaDataProvider.request_new_actor(blueprint, transform, tick=False)
                    if actor is not None:
                        _actors_out.append(actor)
                return py_trees.common.Status.SUCCESS

        seq = py_trees.composites.Sequence("Batch3")
        seq.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self._batch3_trigger,
            distance=DESPAWN_DISTANCE, name="Batch3_SpawnTrigger",
        ))
        seq.add_child(SpawnActorGroup(
            vehicle_spawns=self._batch3_vehicle_spawns,
            walker_spawns=self._batch3_walker_spawns,
            bicycle_spawns=self._batch3_bicycle_spawns,
            actors_out=self._batch3_actors,
            tm_port=tm_port,
            name="Batch3_Spawn",
        ))
        if _static_spawns:
            seq.add_child(_SpawnStaticPedestrians("Batch3_StaticPeds"))
        seq.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self._batch3_despawn,
            distance=DESPAWN_DISTANCE, name="Batch3_DespawnTrigger",
        ))
        seq.add_child(DespawnBatch(self._batch3_actors, name="Batch3_Despawn"))
        seq.add_child(WaitForever())
        return seq

    def _create_batch_despawn_behavior(self, actors, despawn_location, name):
        """
        Returns a Sequence that destroys *actors* once the ego vehicle comes
        within DESPAWN_DISTANCE of *despawn_location*, then waits forever so
        it never triggers an early SUCCESS_ON_ONE on the root Parallel.
        """
        seq = py_trees.composites.Sequence(name)
        seq.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0],
                despawn_location,
                distance=DESPAWN_DISTANCE,
                name="{}_Trigger".format(name),
            )
        )
        seq.add_child(DespawnBatch(actors, name="{}_Destroy".format(name)))
        seq.add_child(WaitForever())
        return seq

    # ------------------------------------------------------------------
    # Play-MP3 guidance branch (own root-level branch)
    # ------------------------------------------------------------------

    def _create_cs2_ped_behavior(self):
        """
        Waits for the ego to pass the construction site 2 trigger point, then
        makes the pedestrian walk quickly to the target position.
        Ends with WaitForever so the branch never causes an early scenario end.
        """
        seq = py_trees.composites.Sequence("CS2_PedEvent")
        seq.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0],
                self._cs2_ped_trigger,
                distance=9.0,
                name="CS2_PedTrigger",
            )
        )
        seq.add_child(
            WalkerWalkTo(
                self._cs2_ped_actor,
                self._cs2_ped_target,
                speed=5.5,
                name="CS2_PedRun",
            )
        )
        seq.add_child(WaitForever())
        return seq

    def _create_play_mp3_behavior(self):
        """
        Build a Sequence that fires PlayMp3 cues one-by-one as the ego
        vehicle approaches each navigation waypoint.

        The branch lives as a direct child of scenario_tree (the root
        Parallel) so it never interferes with the main scenario logic.
        It ends with WaitForever so it never succeeds and cannot trigger
        an early scenario termination.
        """
        play_mp3_root = py_trees.composites.Sequence("PlayMp3Branch")

        # Wait for the ego to actually start moving before checking cues
        play_mp3_root.add_child(DriveDistance(self.ego_vehicles[0], 0.5, name="WaitForEgoToMove"))

        for location, label in _NAVIGATION_CUES:
            cue = py_trees.composites.Sequence("Cue_{}".format(label))
            cue.add_child(
                InTriggerDistanceToLocation(
                    self.ego_vehicles[0],
                    location,
                    distance=CUE_TRIGGER_DISTANCE,
                    name="NearWaypoint_{}".format(label),
                )
            )
            cue.add_child(LogNavigationCue(label, location))
            cue.add_child(PlayMp3(label))
            play_mp3_root.add_child(cue)

        # Keep the branch alive forever so it cannot stop the scenario_tree
        play_mp3_root.add_child(WaitForever())

        return play_mp3_root

    def _create_navigation_markers_behavior(self):
        """
        Build a Sequence that shows gold visual markers one-by-one as the
        ego vehicle approaches each navigation waypoint. Each marker
        disappears when the ego comes within 15 m, and the next one appears.
        """
        seq = py_trees.composites.Sequence("NavigationMarkersBranch")

        # Wait for the ego to actually start moving before showing markers.
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

    # ------------------------------------------------------------------
    # Speed zones — adjust ego max speed at trigger locations
    # ------------------------------------------------------------------

    def _create_speed_zone_behavior(self):
        """
        Sequence that adjusts ego max speed as it passes through speed zones:
          1. Start at 40 km/h (set in __init__).
          2. When ego reaches highway on-ramp area → 90 km/h.
          3. When ego reaches residential area → 40 km/h.
        Ends with WaitForever so the branch never causes an early scenario end.
        """
        seq = py_trees.composites.Sequence("SpeedZoneBranch")

        # Zone 1: highway — increase to 90 km/h
        seq.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            carla.Location(x=385.38, y=-158.36, z=0.00),
            distance=20.0,
            name="SpeedZone_90kmh_Trigger",
        ))
        seq.add_child(SetEgoMaxSpeed(
            self.ego_vehicles[0], 90.0, name="SpeedZone_90kmh",
        ))

        # Zone 2: residential — back to 40 km/h
        seq.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            carla.Location(x=177.17, y=-364.49, z=0.00),
            distance=20.0,
            name="SpeedZone_40kmh_Trigger",
        ))
        seq.add_child(SetEgoMaxSpeed(
            self.ego_vehicles[0], 40.0, name="SpeedZone_40kmh",
        ))

        seq.add_child(WaitForever())
        return seq

    # ------------------------------------------------------------------
    # Criteria
    # ------------------------------------------------------------------

    def _create_test_criteria(self):
        return []  # CollisionTest disabled — flow vehicles may hit ego on highway

    def _set_ego_speed_cap(self, max_speed_kmh):
        """Apply or remove the simulator-side ego max-speed cap."""
        if not self.ego_vehicles or self.ego_vehicles[0] is None:
            print("[AccidentAhead] Ego max speed request skipped: no ego vehicle available")
            return
        ego_vehicle = self.ego_vehicles[0]
        print("[AccidentAhead] Ego speed-cap request: actor_id={} type={} target={:.1f} km/h".format(
            getattr(ego_vehicle, "id", -1),
            getattr(ego_vehicle, "type_id", "unknown"),
            max_speed_kmh,
        ))
        has_set_max_speed = hasattr(ego_vehicle, "set_max_speed")
        print("[AccidentAhead] Ego speed-cap API available: {}".format(has_set_max_speed))
        if not has_set_max_speed:
            print("[AccidentAhead] Python CARLA client does not expose Vehicle.set_max_speed().")
            print("[AccidentAhead] Available related API: get_speed_limit={}".format(
                hasattr(ego_vehicle, "get_speed_limit")))
            return
        try:
            ego_vehicle.set_max_speed(max_speed_kmh)
            print("[AccidentAhead] Ego max speed set to {:.1f} km/h".format(max_speed_kmh))
        except Exception as exc:  # pylint: disable=broad-except
            print("[AccidentAhead] Failed to set ego max speed to {:.1f} km/h: {}".format(
                max_speed_kmh, exc))

    def __del__(self):
        if getattr(self, '_lsl_stream', None):
            self._lsl_stream.stop()
        self._set_ego_speed_cap(0.0)
        self.remove_all_actors()
