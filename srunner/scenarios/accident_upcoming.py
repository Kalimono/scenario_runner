#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Accident Upcoming scenario:

The ego vehicle drives a route through Town04. Audio cues (PlayMp3) guide the
driver at each decision point. The route passes an accident scene on the highway,
construction sites, and a bicycle near-miss event.

Events and locations differ from AccidentAhead — same event types, different order
and positions. Event actors (accident vehicles, construction props, bike) are added
in a second pass once the route is validated.
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
    StopFlowSpawning,
    VehicleDriveAway,
    VehicleFollowPath,
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
#     (carla.Location(x=314.86, y=-210.66, z=0.00), "STRAIGHT"),
#     (carla.Location(x=310.38, y=-290.61, z=0.00), "STRAIGHT"),
#     (carla.Location(x=246.71, y=-311.02, z=0.00), "LEFT"),
#     (carla.Location(x=201.45, y=-294.20, z=0.02), "RIGHT"),
#     (carla.Location(x=149.34, y=-232.87, z=0.04), "RIGHT"),
#     (carla.Location(x=112.02, y=-173.52, z=0.20), "STRAIGHT ENTER HIGHWAY"),
#     (carla.Location(x=16.33,  y=-259.24, z=0.00), "ACCIDENT AHEAD"),
#     (carla.Location(x=340.00, y=-339.01, z=0.00), "STAY ON HIGHWAY UNTIL FURTHER NOTICE"),
#     (carla.Location(x=11.66,  y=-70.22,  z=0.00), "LEAVE NEXT RIGHT"),
#     (carla.Location(x=31.36,  y=-170.48, z=0.20), "STRAIGHT"),
#     (carla.Location(x=83.67,  y=-170.18, z=0.20), "STRAIGHT"),
#     (carla.Location(x=151.63, y=-169.79, z=0.20), "STRAIGHT"),
#     (carla.Location(x=213.60, y=-169.44, z=0.20), "RIGHT"),
#     (carla.Location(x=254.88, y=-154.94, z=0.02), "LEFT"),
# ]

# Visual navigation marker waypoints — gold dots shown sequentially.
_NAVIGATION_MARKERS = [
    carla.Transform(carla.Location(x=315.09, y=-236.22, z=0.00), carla.Rotation(pitch=360.00, yaw=270.51, roll=0.00)),     # 0
    carla.Transform(carla.Location(x=306.42, y=-298.17, z=0.00), carla.Rotation(pitch=360.00, yaw=235.93, roll=0.00)),     # 1
    carla.Transform(carla.Location(x=232.77, y=-311.16, z=0.00), carla.Rotation(pitch=360.00, yaw=180.59, roll=0.00)),     # 2
    carla.Transform(carla.Location(x=201.52, y=-297.25, z=0.02), carla.Rotation(pitch=0.00, yaw=91.31, roll=0.00)),        # 3
    carla.Transform(carla.Location(x=187.39, y=-248.46, z=0.04), carla.Rotation(pitch=0.00, yaw=-183.72, roll=0.00)),      # 4
    carla.Transform(carla.Location(x=117.69, y=-173.49, z=0.20), carla.Rotation(pitch=360.00, yaw=180.33, roll=0.00)),     # 5
    carla.Transform(carla.Location(x=22.41, y=-174.03, z=0.20), carla.Rotation(pitch=360.00, yaw=180.33, roll=0.00)),      # 6
    carla.Transform(carla.Location(x=23.29, y=-287.35, z=0.00), carla.Rotation(pitch=0.00, yaw=-63.67, roll=0.00)),        # 7
    carla.Transform(carla.Location(x=82.32, y=-354.82, z=0.00), carla.Rotation(pitch=0.00, yaw=-29.68, roll=0.00)),        # 8
    carla.Transform(carla.Location(x=303.80, y=-364.63, z=0.00), carla.Rotation(pitch=0.00, yaw=21.75, roll=0.00)),        # 9
    carla.Transform(carla.Location(x=392.80, y=-199.06, z=0.00), carla.Rotation(pitch=0.00, yaw=90.60, roll=0.00)),        # 10
    carla.Transform(carla.Location(x=363.86, y=16.17, z=0.06), carla.Rotation(pitch=0.43, yaw=153.88, roll=0.00)),         # 11
    carla.Transform(carla.Location(x=92.02, y=16.96, z=10.79), carla.Rotation(pitch=0.75, yaw=-539.77, roll=0.00)),        # 12
    carla.Transform(carla.Location(x=-256.16, y=16.10, z=2.05), carla.Rotation(pitch=-2.22, yaw=-179.92, roll=0.00)),      # 13
    carla.Transform(carla.Location(x=-451.04, y=21.30, z=0.00), carla.Rotation(pitch=0.00, yaw=-200.12, roll=0.00)),       # 14
    carla.Transform(carla.Location(x=-503.07, y=265.42, z=0.00), carla.Rotation(pitch=0.00, yaw=82.26, roll=0.00)),        # 15
    carla.Transform(carla.Location(x=-329.46, y=425.25, z=0.00), carla.Rotation(pitch=0.00, yaw=3.01, roll=0.00)),         # 16
    carla.Transform(carla.Location(x=-127.50, y=412.47, z=0.00), carla.Rotation(pitch=0.00, yaw=-20.23, roll=0.00)),       # 17
    carla.Transform(carla.Location(x=-3.44, y=282.39, z=0.00), carla.Rotation(pitch=0.00, yaw=-72.48, roll=0.00)),         # 18
    carla.Transform(carla.Location(x=12.01, y=18.37, z=0.00), carla.Rotation(pitch=0.00, yaw=-90.22, roll=0.00)),          # 19
    carla.Transform(carla.Location(x=14.98, y=-117.47, z=0.00), carla.Rotation(pitch=0.00, yaw=-90.22, roll=0.00)),        # 20
    carla.Transform(carla.Location(x=26.81, y=-170.50, z=0.20), carla.Rotation(pitch=0.00, yaw=0.33, roll=0.00)),          # 21
    carla.Transform(carla.Location(x=150.40, y=-169.80, z=0.20), carla.Rotation(pitch=0.00, yaw=0.33, roll=0.00)),         # 22
    carla.Transform(carla.Location(x=239.14, y=-169.30, z=0.20), carla.Rotation(pitch=0.00, yaw=0.33, roll=0.00)),         # 23
    carla.Transform(carla.Location(x=254.82, y=-135.93, z=0.02), carla.Rotation(pitch=360.00, yaw=90.18, roll=0.00)),      # 24
    carla.Transform(carla.Location(x=285.71, y=-118.38, z=0.02), carla.Rotation(pitch=0.00, yaw=0.92, roll=0.00)),         # 25
]

CUE_TRIGGER_DISTANCE = 20   # metres; wider tolerance so cues survive small route deviations
TRAFFIC_BATCH_SEED   = 42   # fixed seed for reproducible autopilot behaviour
_DEBUG_DRAW_MARKINGS = False
_DEBUG_DRAW_SOUNDCUES = False
_DEBUG_DRAW_NAV_MARKERS = False
_DEBUG_DRAW_PARKED_MARKINGS = False
_DEBUG_DRAW_BIKE_TRIGGER = False
_DEBUG_DRAW_FINAL_BIKE_EVENT = False
_DEBUG_FIRST_BIKE_ONLY = False
_DEBUG_DRAW_FIRST_BIKE_EVENT = False


class AccidentUpcoming(BasicScenario):
    """
    Ego vehicle drives a guided route past an accident scene, construction sites,
    and a bicycle near-miss event (events to be positioned in a second pass).

    The scenario ends when the ego vehicle reaches the final destination.
    """

    timeout = 9999999

    @staticmethod
    def _is_high_load_config(config):
        """
        Infer the cognitive-load variant from any scenario metadata that may
        carry the variant name. This keeps the behavior stable even when the
        scenario is launched through a route wrapper or renamed config.
        """
        candidate_fields = [
            getattr(config, "name", None),
            getattr(config, "subtype", None),
            getattr(config, "route_var_name", None),
        ]

        other_parameters = getattr(config, "other_parameters", {}) or {}
        candidate_fields.extend(other_parameters.keys())
        candidate_fields.extend(other_parameters.values())

        for value in candidate_fields:
            if value is None:
                continue
            if "high" in str(value).lower():
                return True
        return False

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=9999999):
        self._map   = CarlaDataProvider.get_map()
        self._world = world
        self.timeout = timeout

        # ------------------------------------------------------------------
        # Accident scene
        # ------------------------------------------------------------------
        self._accident_actor_spawns = [
            ("vehicle.dodge.charger_2020", carla.Transform(
                carla.Location(x=199.66, y=-365.61, z=1.00),
                carla.Rotation(pitch=0.00, yaw=8.05, roll=0.00),
            )),
            ("vehicle.ford.ambulance", carla.Transform(
                carla.Location(x=205.52, y=-357.57, z=1.00),
                carla.Rotation(pitch=0.00, yaw=63.99, roll=0.00),
            )),
            ("vehicle.ford.ambulance", carla.Transform(
                carla.Location(x=188.95, y=-364.49, z=1.60),
                carla.Rotation(pitch=0.00, yaw=174.17, roll=0.00),
            )),
            ("vehicle.dodge.charger_police", carla.Transform(
                carla.Location(x=203.09, y=-317.20, z=0.02),
                carla.Rotation(pitch=0.00, yaw=18.69, roll=0.00),
            )),
            ("vehicle.dodge.charger_police", carla.Transform(
                carla.Location(x=188.62, y=-370.31, z=1.00),
                carla.Rotation(pitch=0.00, yaw=77.08, roll=0.00),
            )),
            ("vehicle.dodge.charger_police", carla.Transform(
                carla.Location(x=208.99, y=-368.93, z=0.80),
                carla.Rotation(pitch=0.00, yaw=157.64, roll=0.00),
            )),
        ]

        # Event ambulance — spawned separately so it can be driven away.
        self._event_ambulance_spawn = carla.Transform(
            carla.Location(x=195.72, y=-369.81, z=1.00),
            carla.Rotation(pitch=0.00, yaw=-11.90, roll=0.00),
        )
        self._event_ambulance = None
        self._ambulance_trigger_location = carla.Location(x=190.89, y=-374.88, z=0.00)
        self._ambulance_target_location = carla.Location(x=215.82, y=-374.70, z=0.00)
        self._accident_scene_actors = []
        self._pedestrian_spawns = []

        # ------------------------------------------------------------------
        # Accident scene barriers (static props, placed at exact captured z).
        # ------------------------------------------------------------------
        self._accident_barrier_spawns = []

        # ------------------------------------------------------------------
        # Construction sites — TODO: place barriers/cones once route is validated
        # ------------------------------------------------------------------
        self._construction_barrier_spawns = []
        self._construction_cone_spawns    = []

        # ------------------------------------------------------------------
        # Bicycle near-miss event
        # Bike spawns and rides west along the road toward the ego's path.
        # Speed is adjusted so bike reaches _bike_sync_transform at the same
        # time ego reaches _bike_ego_sync_transform.
        # ------------------------------------------------------------------
        self._bike_spawn_transform = carla.Transform(
            carla.Location(x=109.57, y=-183.67, z=0.34),
            carla.Rotation(pitch=0.00, yaw=-165.90, roll=0.00),
        )
        self._bike_sync_transform = carla.Transform(
            carla.Location(x=127.64, y=-179.67, z=0.04),
            carla.Rotation(pitch=0.00, yaw=-267.54, roll=0.00),
        )
        self._bike_end_transform = carla.Transform(
            carla.Location(x=144.99, y=-179.14, z=0.35),
            carla.Rotation(pitch=0.00, yaw=-176.47, roll=0.00),
        )
        # Trigger: ego reaches this location → bike event starts.
        self._bike_trigger_location = carla.Location(x=136.66, y=-207.22, z=0.04)
        # Ego sync point: ego should arrive here at the same time bike reaches _bike_sync_transform.
        self._bike_ego_sync_transform = carla.Transform(
            carla.Location(x=127.64, y=-179.67, z=0.04),
            carla.Rotation(pitch=0.00, yaw=-267.54, roll=0.00),
        )

        # ------------------------------------------------------------------
        # Final event series — car, bicycle, pedestrian near the end of route
        # ------------------------------------------------------------------
        self._event_car = None

        # Sub-event 2: Bicycle rides onto the road.
        self._event_bike_spawn = carla.Transform(
            carla.Location(x=227.93, y=-186.68, z=0.27),
            carla.Rotation(pitch=0.00, yaw=90.16, roll=0.00),
        )
        self._event_bike_trigger = carla.Location(x=175.08, y=-169.55, z=0.20)
        self._event_bike_target  = carla.Location(x=227.91, y=-172.86, z=0.20)
        self._event_bike = None

        # Sub-event 3: Pedestrian runs into the road in shock.
        self._event_ped_spawn = carla.Transform(
            carla.Location(x=245.07, y=-173.51, z=0.20),
            carla.Rotation(pitch=0.00, yaw=118.45, roll=0.00),
        )
        self._event_ped_trigger = carla.Location(x=237.59, y=-169.40, z=0.20)
        self._event_ped_target  = carla.Location(x=243.34, y=-169.99, z=0.20)
        self._event_ped = None

        # ------------------------------------------------------------------
        # Traffic flows
        # ------------------------------------------------------------------
        # Flow A — rightmost lane, highway speed.
        self._flow_a_source_loc = carla.Location(x=310.39, y=-101.87, z=0.00)
        self._flow_a_sink_loc   = carla.Location(x=86.72,  y=-345.24, z=0.00)
        # Flow B — second lane from right, slightly faster.
        self._flow_b_source_loc = carla.Location(x=-350.24, y=-69.93, z=0.00)
        self._flow_b_sink_loc   = carla.Location(x=84.04,   y=-347.73, z=0.00)
        # Both flows start when ego reaches this trigger.
        self._flow_trigger_loc  = carla.Location(x=169.02, y=-244.53, z=0.04)
        # Flows are fully cleared when ego reaches the batch 2 despawn trigger.
        self._flow_stop_loc     = carla.Location(x=289.59, y=-118.32, z=0.02)

        # ------------------------------------------------------------------
        # Batch 1 — ambient traffic, spawns at scenario start.
        # Despawns when ego reaches the construction site area.
        # ------------------------------------------------------------------
        self._batch1_walker_spawns = [
            ("walker.pedestrian.0001", carla.Transform(
                carla.Location(x=221.34, y=-303.89, z=0.00),
                carla.Rotation(pitch=0.00, yaw=0.59, roll=0.00),
            )),
            ("walker.pedestrian.0002", carla.Transform(
                carla.Location(x=286.73, y=-302.91, z=0.00),
                carla.Rotation(pitch=0.00, yaw=10.46, roll=0.00),
            )),
        ]
        self._batch1_bicycle_spawns = []
        self._batch1_vehicle_spawns = [
            ("vehicle.audi.tt", carla.Transform(
                carla.Location(x=311.84, y=-267.61, z=0.00),
                carla.Rotation(pitch=0.00, yaw=88.86, roll=0.00),
            )),
            ("vehicle.chevrolet.impala", carla.Transform(
                carla.Location(x=314.87, y=-212.05, z=0.00),
                carla.Rotation(pitch=360.00, yaw=270.51, roll=0.00),
            )),
            ("vehicle.nissan.patrol", carla.Transform(
                carla.Location(x=131.37, y=-185.04, z=0.04),
                carla.Rotation(pitch=360.00, yaw=272.46, roll=0.00),
            )),
        ]
        self._batch1_despawn  = carla.Location(x=14.65, y=-200.74, z=0.00)
        self._batch1_actors   = []
        self._batch1_static_vehicle_spawns = []
        self._batch1_static_pedestrian_spawns = []
        self._batch1_static_cone_spawns = []

        # ------------------------------------------------------------------
        # Batch 2 - car-only traffic, spawned when batch 1 despawns.
        # Despawns when ego reaches the highway exit area. Active flows are
        # stopped and fully cleared at the same trigger.
        # ------------------------------------------------------------------
        self._batch2_trigger = self._batch1_despawn
        self._batch2_vehicle_spawns = [
            ("vehicle.audi.tt", carla.Transform(
                carla.Location(x=151.04, y=35.39, z=9.31),
                carla.Rotation(pitch=357.89, yaw=0.98, roll=0.00),
            )),
        ]
        self._batch2_despawn = carla.Location(x=11.66, y=-71.04, z=0.00)
        self._batch2_actors = []
        self._flow_stop_loc = self._batch2_despawn

        # ------------------------------------------------------------------
        # Batch 3 - mixed urban traffic, spawned immediately after batch 2
        # despawns and the flows are cleared.
        # ------------------------------------------------------------------
        self._batch3_trigger = carla.Location(x=11.69, y=-64.41, z=0.00)
        self._batch3_walker_spawns = [
            ("walker.pedestrian.0001", carla.Transform(
                carla.Location(x=101.54, y=-179.46, z=0.20),
                carla.Rotation(pitch=360.00, yaw=180.33, roll=0.00),
            )),
        ]
        self._batch3_bicycle_spawns = []
        self._batch3_vehicle_spawns = [
            ("vehicle.audi.tt", carla.Transform(
                carla.Location(x=51.44, y=-173.86, z=0.20),
                carla.Rotation(pitch=0.00, yaw=-179.67, roll=0.00),
            )),
            ("vehicle.chevrolet.impala", carla.Transform(
                carla.Location(x=186.11, y=-173.10, z=0.20),
                carla.Rotation(pitch=360.00, yaw=180.33, roll=0.00),
            )),
        ]
        self._batch3_static_vehicle_spawns = [
            ("vehicle.mitsubishi.fusorosa", carla.Transform(
                carla.Location(x=237.65, y=-172.80, z=0.2),
                carla.Rotation(pitch=360.00, yaw=180.33, roll=0.00),
            ), carla.VehicleLightState.RightBlinker, True),
        ]
        self._batch3_actors = []

        # ------------------------------------------------------------------
        # Cognitive load variant
        # ------------------------------------------------------------------
        self._batch3_static_pedestrian_spawns = []
        self._high_load = self._is_high_load_config(config)
        if self._high_load:
            self._batch1_vehicle_spawns.extend([
                ("vehicle.lincoln.mkz_2020", carla.Transform(
                    carla.Location(x=213.49, y=-245.92, z=0.00),
                    carla.Rotation(pitch=360.00, yaw=359.61, roll=0.00))),
                ("vehicle.dodge.charger_2020", carla.Transform(
                    carla.Location(x=264.83, y=-310.83, z=0.00),
                    carla.Rotation(pitch=360.00, yaw=180.59, roll=0.00))),
                ("vehicle.ford.mustang", carla.Transform(
                    carla.Location(x=217.82, y=-311.31, z=0.00),
                    carla.Rotation(pitch=360.00, yaw=180.59, roll=0.00))),
                ("vehicle.seat.leon", carla.Transform(
                    carla.Location(x=155.00, y=-183.50, z=0.04),
                    carla.Rotation(pitch=360.00, yaw=272.46, roll=0.00))),
            ])
            self._batch1_walker_spawns.extend([
                ("walker.pedestrian.0007", carla.Transform(
                    carla.Location(x=195.35, y=-284.79, z=0.02),
                    carla.Rotation(pitch=0.00, yaw=91.31, roll=0.00))),
                ("walker.pedestrian.0008", carla.Transform(
                    carla.Location(x=208.41, y=-275.60, z=0.02),
                    carla.Rotation(pitch=0.00, yaw=271.31, roll=0.00))),
                ("walker.pedestrian.0009", carla.Transform(
                    carla.Location(x=229.64, y=-303.81, z=0.00),
                    carla.Rotation(pitch=0.00, yaw=0.59, roll=0.00))),
                ("walker.pedestrian.0010", carla.Transform(
                    carla.Location(x=281.54, y=-253.78, z=0.00),
                    carla.Rotation(pitch=0.00, yaw=179.61, roll=0.00))),
                ("walker.pedestrian.0011", carla.Transform(
                    carla.Location(x=284.23, y=-242.52, z=0.00),
                    carla.Rotation(pitch=360.00, yaw=359.61, roll=0.00))),
                ("walker.pedestrian.0012", carla.Transform(
                    carla.Location(x=307.65, y=-229.95, z=0.00),
                    carla.Rotation(pitch=0.00, yaw=90.51, roll=0.00))),
                ("walker.pedestrian.0013", carla.Transform(
                    carla.Location(x=307.48, y=-210.99, z=0.00),
                    carla.Rotation(pitch=0.00, yaw=90.51, roll=0.00))),
            ])
            self._batch2_vehicle_spawns.extend([
                ("vehicle.chevrolet.impala", carla.Transform(
                    carla.Location(x=120.48, y=20.15, z=9.31),
                    carla.Rotation(pitch=357.89, yaw=0.98, roll=0.00))),
            ])
            self._batch3_vehicle_spawns.extend([
                ("vehicle.audi.tt", carla.Transform(
                    carla.Location(x=91.40, y=-173.64, z=0.20),
                    carla.Rotation(pitch=360.00, yaw=180.33, roll=0.00))),
                ("vehicle.nissan.patrol", carla.Transform(
                    carla.Location(x=170.84, y=-173.18, z=0.20),
                    carla.Rotation(pitch=0.00, yaw=180.33, roll=0.00))),
            ])
            self._batch3_walker_spawns.extend([
                ("walker.pedestrian.0005", carla.Transform(
                    carla.Location(x=137.15, y=-182.42, z=0.04),
                    carla.Rotation(pitch=360.00, yaw=272.46, roll=0.00))),
                ("walker.pedestrian.0006", carla.Transform(
                    carla.Location(x=106.16, y=-179.44, z=0.20),
                    carla.Rotation(pitch=360.00, yaw=180.33, roll=0.00))),
                ("walker.pedestrian.0023", carla.Transform(
                    carla.Location(x=88.18, y=-179.54, z=0.20),
                    carla.Rotation(pitch=360.00, yaw=180.33, roll=0.00))),
                ("walker.pedestrian.0024", carla.Transform(
                    carla.Location(x=52.78, y=-184.63, z=0.03),
                    carla.Rotation(pitch=0.00, yaw=90.35, roll=0.00))),
                ("walker.pedestrian.0025", carla.Transform(
                    carla.Location(x=206.37, y=-154.46, z=0.02),
                    carla.Rotation(pitch=360.00, yaw=260.42, roll=0.00))),
            ])
            self._batch3_bicycle_spawns.extend([
                ("vehicle.bh.crossbike", carla.Transform(
                    carla.Location(x=113.31, y=-173.51, z=0.20),
                    carla.Rotation(pitch=360.00, yaw=180.33, roll=0.00))),
                ("vehicle.diamondback.century", carla.Transform(
                    carla.Location(x=103.39, y=-173.57, z=0.20),
                    carla.Rotation(pitch=360.00, yaw=180.33, roll=0.00))),
            ])
            # Extra static props — parked cars (spawned and frozen).
            self._batch1_static_vehicle_spawns = []
            self._batch1_static_cone_spawns = [
                carla.Transform(carla.Location(x=195.10, y=-370.20, z=0.00), carla.Rotation(yaw=0.0)),
                carla.Transform(carla.Location(x=195.10, y=-362.20, z=0.00), carla.Rotation(yaw=0.0)),
                carla.Transform(carla.Location(x=195.10, y=-354.20, z=0.00), carla.Rotation(yaw=0.0)),
            ]
            self._batch1_static_pedestrian_spawns = [
                ("walker.pedestrian.0014", carla.Transform(carla.Location(x=251.60, y=-300.79, z=1.67), carla.Rotation(pitch=0.00, yaw=-15.86, roll=0.00))),
                ("walker.pedestrian.0015", carla.Transform(carla.Location(x=251.94, y=-298.82, z=1.67), carla.Rotation(pitch=0.00, yaw=2.84, roll=0.00))),
                ("walker.pedestrian.0016", carla.Transform(carla.Location(x=196.87, y=-281.56, z=1.67), carla.Rotation(pitch=0.00, yaw=-3.98, roll=0.00))),
                ("walker.pedestrian.0017", carla.Transform(carla.Location(x=196.30, y=-271.08, z=1.67), carla.Rotation(pitch=0.00, yaw=0.92, roll=0.00))),
                ("walker.pedestrian.0018", carla.Transform(carla.Location(x=192.85, y=-253.95, z=1.69), carla.Rotation(pitch=0.00, yaw=8.79, roll=0.00))),
            ]
            self._batch3_static_pedestrian_spawns = [
                ("walker.pedestrian.0019", carla.Transform(carla.Location(x=138.90, y=-197.76, z=1.69), carla.Rotation(pitch=0.00, yaw=99.06, roll=0.00))),
                ("walker.pedestrian.0020", carla.Transform(carla.Location(x=138.35, y=-195.14, z=1.69), carla.Rotation(pitch=0.00, yaw=-75.60, roll=0.00))),
                ("walker.pedestrian.0021", carla.Transform(carla.Location(x=121.39, y=-191.10, z=1.69), carla.Rotation(pitch=0.00, yaw=-15.74, roll=0.00))),
                ("walker.pedestrian.0022", carla.Transform(carla.Location(x=121.63, y=-193.09, z=1.69), carla.Rotation(pitch=0.00, yaw=6.79, roll=0.00))),
                ("walker.pedestrian.0026", carla.Transform(carla.Location(x=142.27, y=-179.52, z=1.85), carla.Rotation(pitch=0.00, yaw=88.46, roll=0.00))),
                ("walker.pedestrian.0027", carla.Transform(carla.Location(x=145.20, y=-179.60, z=1.85), carla.Rotation(pitch=0.00, yaw=88.46, roll=0.00))),
                ("walker.pedestrian.0028", carla.Transform(carla.Location(x=137.47, y=-185.52, z=1.69), carla.Rotation(pitch=0.00, yaw=176.72, roll=0.00))),
                ("walker.pedestrian.0029", carla.Transform(carla.Location(x=72.18, y=-186.02, z=1.83), carla.Rotation(pitch=0.00, yaw=-90.00, roll=0.00))),
                ("walker.pedestrian.0030", carla.Transform(carla.Location(x=72.46, y=-188.94, z=1.82), carla.Rotation(pitch=0.00, yaw=92.86, roll=0.00))),
                ("walker.pedestrian.0031", carla.Transform(carla.Location(x=207.13, y=-179.25, z=2.24), carla.Rotation(pitch=0.00, yaw=57.69, roll=0.00))),
                ("walker.pedestrian.0032", carla.Transform(carla.Location(x=208.32, y=-177.45, z=2.24), carla.Rotation(pitch=0.00, yaw=-127.25, roll=0.00))),
                ("walker.pedestrian.0033", carla.Transform(carla.Location(x=239.89, y=-176.70, z=1.85), carla.Rotation(pitch=0.00, yaw=92.61, roll=0.00))),
                ("walker.pedestrian.0034", carla.Transform(carla.Location(x=238.06, y=-176.78, z=1.85), carla.Rotation(pitch=0.00, yaw=92.61, roll=0.00))),
            ]
        else:
            # Low cognitive load — fewer actors.
            self._batch2_vehicle_spawns = []
            self._batch3_walker_spawns = self._batch3_walker_spawns[:1]
            self._batch3_bicycle_spawns = self._batch3_bicycle_spawns[:1]
            self._batch1_static_vehicle_spawns = []
            self._batch1_static_pedestrian_spawns = []
            self._batch1_static_cone_spawns = []
            self._batch3_static_pedestrian_spawns = []

        # ------------------------------------------------------------------
        # Scenario end location
        # ------------------------------------------------------------------
        self._end_location = carla.Location(x=289.59, y=-118.32, z=0.02)
        self._batch1_walker_controllers = []

        print(
            "[AU] load={} batch1(v={}, w={}, b={}) batch2(v={}) "
            "batch3(v={}, w={}, b={}, static_w={}) batch1_static(parked={}, cones={}, static_w={})".format(
                "HIGH" if self._high_load else "LOW",
                len(self._batch1_vehicle_spawns),
                len(self._batch1_walker_spawns),
                len(self._batch1_bicycle_spawns),
                len(self._batch2_vehicle_spawns),
                len(self._batch3_vehicle_spawns),
                len(self._batch3_walker_spawns),
                len(self._batch3_bicycle_spawns),
                len(self._batch3_static_vehicle_spawns),
                len(self._batch3_static_pedestrian_spawns),
                len(self._batch1_static_vehicle_spawns),
                len(self._batch1_static_cone_spawns),
                len(self._batch1_static_pedestrian_spawns),
            )
        )

        super(AccidentUpcoming, self).__init__(
            "AccidentUpcoming",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=False,
        )

        # Initial 40 km/h speed cap.
        ego = self.ego_vehicles[0]
        if hasattr(ego, 'set_max_speed'):
            ego.set_max_speed(40.0)
            print("[AccidentUpcoming] Ego max speed set to 40 km/h")

        # Fixed seed so pedestrian navigation destinations are reproducible.
        world.set_pedestrians_seed(TRAFFIC_BATCH_SEED)

        # Fixed TM seed so autopilot traffic follows the same choices each run.
        tm_port = CarlaDataProvider.get_traffic_manager_port()
        tm = CarlaDataProvider.get_client().get_trafficmanager(tm_port)
        tm.set_random_device_seed(TRAFFIC_BATCH_SEED)

        # Two ticks are needed between controller spawn and start()
        # (matches the pattern in generate_traffic.py). super().__init__()
        # already provided one tick; add a second here.
        world.tick()
        for controller in self._batch1_walker_controllers:
            controller.start()
            dest = world.get_random_location_from_navigation()
            if dest is not None:
                controller.go_to_location(dest)
            controller.set_max_speed(1.4)

        root_tree = self.scenario_tree

        if _DEBUG_FIRST_BIKE_ONLY:
            bike_branch = self._create_bike_behavior()
            root_tree.add_child(bike_branch)
            bike_branch.setup(timeout=1)
            self._draw_first_bike_event_markers(world)
            self.scenario_tree.setup(timeout=1)
            return

        lights_branch = KeepTrafficLightsGreen()
        root_tree.add_child(lights_branch)
        lights_branch.setup(1)

        # Speed zone branch — adjusts ego max speed at trigger locations.
        speed_zone_branch = self._create_speed_zone_behavior()
        root_tree.add_child(speed_zone_branch)
        speed_zone_branch.setup(timeout=1)

        # Ego vehicle telemetry → LSL stream (background thread, not in behavior tree).
        self._lsl_stream = EgoVehicleLSLStream(
            self.ego_vehicles[0],
            participant_id=getattr(config, 'participant_id', ''))
        self._lsl_stream.start()

        # Batch 1 despawn branch — destroys ambient traffic when ego reaches despawn point.
        batch1_branch = self._create_batch_despawn_behavior(
            self._batch1_actors, self._batch1_despawn, "Batch1Despawn"
        )
        root_tree.add_child(batch1_branch)
        batch1_branch.setup(timeout=1)

        batch2_branch = self._create_triggered_vehicle_batch_behavior(
            self._batch2_vehicle_spawns,
            self._batch2_actors,
            self._batch2_trigger,
            self._batch2_despawn,
            tm_port,
            "Batch2",
        )
        root_tree.add_child(batch2_branch)
        batch2_branch.setup(timeout=1)

        accident_scene_branch = self._create_batch_despawn_behavior(
            self._accident_scene_actors, self._batch2_despawn, "AccidentSceneDespawn"
        )
        root_tree.add_child(accident_scene_branch)
        accident_scene_branch.setup(timeout=1)

        batch3_branch = self._create_triggered_mixed_batch_spawn_behavior(
            self._batch3_vehicle_spawns,
            self._batch3_walker_spawns,
            self._batch3_bicycle_spawns,
            self._batch3_actors,
            self._batch3_trigger,
            tm_port,
            "Batch3",
            static_vehicle_spawns=self._batch3_static_vehicle_spawns,
            static_pedestrian_spawns=self._batch3_static_pedestrian_spawns,
        )
        root_tree.add_child(batch3_branch)
        batch3_branch.setup(timeout=1)


        # Audio guidance branch — commented out, replaced by visual markers.
        # play_mp3_branch = self._create_play_mp3_behavior()
        # root_tree.add_child(play_mp3_branch)
        # play_mp3_branch.setup(timeout=1)

        # Visual navigation markers — gold dots shown one at a time.
        nav_markers_branch = self._create_navigation_markers_behavior()
        root_tree.add_child(nav_markers_branch)
        nav_markers_branch.setup(timeout=1)

        # Bicycle near-miss branch.
        bike_branch = self._create_bike_behavior()
        root_tree.add_child(bike_branch)
        bike_branch.setup(timeout=1)

        # Ambulance drive-away event branch.
        if self._event_ambulance is not None:
            ambulance_branch = self._create_ambulance_event()
            root_tree.add_child(ambulance_branch)
            ambulance_branch.setup(timeout=1)

        # ------------------------------------------------------------------
        # Traffic flows
        # ------------------------------------------------------------------
        # Flow A - rightmost driving lane.
        a_source_wp = self._map.get_waypoint(self._flow_a_source_loc, project_to_road=True,
                                             lane_type=carla.LaneType.Driving)
        a_sink_wp = self._map.get_waypoint(self._flow_a_sink_loc, project_to_road=True,
                                           lane_type=carla.LaneType.Driving)
        if a_source_wp and a_sink_wp:
            flow_a = ActorFlowSections(
                a_source_wp, a_sink_wp,
                spawn_dist_interval=(400, 500),
                sections=[a_source_wp.transform.location, a_sink_wp.transform.location],
                actor_speed=90 / 3.6,
                initial_actors=False,
                allow_lane_change=True,
                name="FlowA",
            )
            branch_a = self._flow_branch_triggered(flow_a, self._flow_trigger_loc, self._flow_stop_loc)
            root_tree.add_child(branch_a)
            branch_a.setup(timeout=1)
        else:
            print("[AU] WARNING: FlowA skipped (waypoint not found)")

        # Flow B - second lane from right.
        b_source_wp = self._map.get_waypoint(self._flow_b_source_loc, project_to_road=True,
                                             lane_type=carla.LaneType.Driving)
        b_sink_wp = self._map.get_waypoint(self._flow_b_sink_loc, project_to_road=True,
                                           lane_type=carla.LaneType.Driving)
        if b_source_wp:
            next_lane = b_source_wp.get_left_lane()
            if next_lane and next_lane.lane_type == carla.LaneType.Driving:
                b_source_wp = next_lane
                print("[AU] FlowB source moved to lane_id={}".format(b_source_wp.lane_id))
        if b_source_wp and b_sink_wp:
            flow_b = ActorFlowSections(
                b_source_wp, b_sink_wp,
                spawn_dist_interval=(400, 500),
                sections=[b_source_wp.transform.location, b_sink_wp.transform.location],
                actor_speed=100 / 3.6,
                initial_actors=False,
                allow_lane_change=True,
                name="FlowB",
            )
            branch_b = self._flow_branch_triggered(flow_b, self._flow_trigger_loc, self._flow_stop_loc)
            root_tree.add_child(branch_b)
            branch_b.setup(timeout=1)
        else:
            print("[AU] WARNING: FlowB skipped (waypoint not found)")

        # ------------------------------------------------------------------
        # Final event series — car, bicycle, pedestrian.
        # ------------------------------------------------------------------
        if self._event_car is not None:
            car_branch = self._create_event_car()
            root_tree.add_child(car_branch)
            car_branch.setup(timeout=1)

        if self._event_bike is not None:
            bike2_branch = self._create_event_bike()
            root_tree.add_child(bike2_branch)
            bike2_branch.setup(timeout=1)

        if self._event_ped is not None:
            ped_branch = self._create_event_ped()
            root_tree.add_child(ped_branch)
            ped_branch.setup(timeout=1)

        # Freeze all traffic lights to green.
        for tl in world.get_actors().filter("traffic.traffic_light*"):
            tl.set_state(carla.TrafficLightState.Green)
            tl.set_green_time(99999.0)
            tl.set_red_time(0.0)
            tl.set_yellow_time(0.0)
            tl.freeze(True)

        if _DEBUG_DRAW_SOUNDCUES:
            # Draw navigation cue markers on the map.
            # Each cue gets a point + two labels: the instruction and the audio filename.
            _cue_color = carla.Color(255, 0, 255)
            for i, (location, label) in enumerate(_NAVIGATION_CUES):
                filename = label.lower().replace(" ", "_") + ".wav"
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
                    color=carla.Color(255, 80, 255),
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

        if _DEBUG_DRAW_PARKED_MARKINGS:
            _parked_color = carla.Color(80, 200, 255)
            for i, (_, transform) in enumerate(self._batch1_static_vehicle_spawns):
                loc = transform.location
                world.debug.draw_point(
                    carla.Location(x=loc.x, y=loc.y, z=loc.z + 1.0),
                    size=0.25,
                    color=_parked_color,
                    life_time=9999,
                )
                world.debug.draw_string(
                    carla.Location(x=loc.x, y=loc.y, z=loc.z + 2.5),
                    "PK{}".format(i + 1),
                    draw_shadow=True,
                    color=_parked_color,
                    life_time=9999,
                    persistent_lines=True,
                )

        if _DEBUG_DRAW_BIKE_TRIGGER:
            _bike_trigger_color = carla.Color(255, 120, 0)
            base = self._bike_trigger_location
            for dz in [0.5, 1.5, 3.0]:
                world.debug.draw_point(
                    carla.Location(x=base.x, y=base.y, z=base.z + dz),
                    size=0.25,
                    color=_bike_trigger_color,
                    life_time=9999,
                )
            world.debug.draw_string(
                carla.Location(x=base.x, y=base.y, z=base.z + 4.5),
                "BIKE TRIGGER",
                draw_shadow=True,
                color=_bike_trigger_color,
                life_time=9999,
                persistent_lines=True,
            )

        self._draw_first_bike_event_markers(world)

        if _DEBUG_DRAW_FINAL_BIKE_EVENT:
            _event_bike_color = carla.Color(0, 180, 255)
            _event_bike_target_color = carla.Color(255, 210, 0)
            _event_bike_trigger_color = carla.Color(255, 80, 80)

            _spawn = self._event_bike_spawn.location
            for dz in [0.5, 1.5, 3.0]:
                world.debug.draw_point(
                    carla.Location(x=_spawn.x, y=_spawn.y, z=_spawn.z + dz),
                    size=0.25,
                    color=_event_bike_color,
                    life_time=9999,
                )
            world.debug.draw_string(
                carla.Location(x=_spawn.x, y=_spawn.y, z=_spawn.z + 4.5),
                "FINAL BIKE SPAWN",
                draw_shadow=True,
                color=_event_bike_color,
                life_time=9999,
                persistent_lines=True,
            )

            _target = self._event_bike_target
            for dz in [0.5, 1.5, 3.0]:
                world.debug.draw_point(
                    carla.Location(x=_target.x, y=_target.y, z=_target.z + dz),
                    size=0.25,
                    color=_event_bike_target_color,
                    life_time=9999,
                )
            world.debug.draw_string(
                carla.Location(x=_target.x, y=_target.y, z=_target.z + 4.5),
                "FINAL BIKE TARGET",
                draw_shadow=True,
                color=_event_bike_target_color,
                life_time=9999,
                persistent_lines=True,
            )

            _trigger = self._event_bike_trigger
            for dz in [0.5, 1.5, 3.0]:
                world.debug.draw_point(
                    carla.Location(x=_trigger.x, y=_trigger.y, z=_trigger.z + dz),
                    size=0.25,
                    color=_event_bike_trigger_color,
                    life_time=9999,
                )
            world.debug.draw_string(
                carla.Location(x=_trigger.x, y=_trigger.y, z=_trigger.z + 4.5),
                "FINAL BIKE EGO TRIGGER",
                draw_shadow=True,
                color=_event_bike_trigger_color,
                life_time=9999,
                persistent_lines=True,
            )

            _ped_trigger = self._event_ped_trigger
            for dz in [0.5, 1.5, 3.0]:
                world.debug.draw_point(
                    carla.Location(x=_ped_trigger.x, y=_ped_trigger.y, z=_ped_trigger.z + dz),
                    size=0.25,
                    color=_event_bike_trigger_color,
                    life_time=9999,
                )
            world.debug.draw_string(
                carla.Location(x=_ped_trigger.x, y=_ped_trigger.y, z=_ped_trigger.z + 4.5),
                "FINAL PED EGO TRIGGER",
                draw_shadow=True,
                color=_event_bike_trigger_color,
                life_time=9999,
                persistent_lines=True,
            )

        # BasicScenario sets up the tree before these extra branches are attached.
        # Re-setup once at the end so the full augmented tree is definitely live.
        self.scenario_tree.setup(timeout=1)

    def _draw_first_bike_event_markers(self, world):
        """Draw the first bike near-miss geometry for debugging."""
        if not _DEBUG_DRAW_FIRST_BIKE_EVENT:
            return
        _first_bike_markers = [
            (self._bike_spawn_transform.location, "FIRST BIKE SPAWN", carla.Color(0, 180, 255)),
            (self._bike_sync_transform.location, "FIRST BIKE SYNC", carla.Color(255, 210, 0)),
            (self._bike_end_transform.location, "FIRST BIKE END", carla.Color(120, 255, 120)),
            (self._bike_trigger_location, "FIRST BIKE EGO TRIGGER", carla.Color(255, 80, 80)),
            (self._bike_ego_sync_transform.location, "FIRST BIKE EGO SYNC", carla.Color(255, 120, 255)),
        ]
        for location, label, color in _first_bike_markers:
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

    # ------------------------------------------------------------------
    # Actor initialisation
    # ------------------------------------------------------------------

    def _initialize_actors(self, config):
        """Spawn scene actors. Accident vehicles are placed above ground; gravity settles them."""
        self._initialize_actors_impl(config)

    def _initialize_actors_impl(self, config):
        _EMERGENCY_LIGHTS = carla.VehicleLightState(
            carla.VehicleLightState.Special1 | carla.VehicleLightState.Special2
        )

        for blueprint, transform in self._accident_actor_spawns:
            actor = CarlaDataProvider.request_new_actor(blueprint, transform)
            if actor is not None:
                actor.set_simulate_physics(enabled=True)
                if "ambulance" in blueprint or "police" in blueprint or "firetruck" in blueprint:
                    actor.set_light_state(_EMERGENCY_LIGHTS)
                self._accident_scene_actors.append(actor)
                self.other_actors.append(actor)

        # Event ambulance — tracked separately for the drive-away event.
        actor = CarlaDataProvider.request_new_actor("vehicle.ford.ambulance", self._event_ambulance_spawn)
        if actor is not None:
            actor.set_simulate_physics(enabled=True)
            actor.set_light_state(_EMERGENCY_LIGHTS)
            self._event_ambulance = actor
            self._accident_scene_actors.append(actor)
            self.other_actors.append(actor)

        for blueprint, transform in self._pedestrian_spawns:
            actor = CarlaDataProvider.request_new_actor(blueprint, transform)
            if actor is not None:
                self.other_actors.append(actor)

        # Batch 1 — ambient traffic at scenario start.
        tm_port = CarlaDataProvider.get_traffic_manager_port()
        for blueprint, transform in self._batch1_vehicle_spawns + self._batch1_bicycle_spawns:
            actor = CarlaDataProvider.request_new_actor(
                blueprint,
                transform,
                rolename='autopilot',
            )
            if actor is not None:
                actor.set_autopilot(True, tm_port)
                self._batch1_actors.append(actor)

        controller_bp = self._world.get_blueprint_library().find('controller.ai.walker')
        for blueprint, transform in self._batch1_walker_spawns:
            actor = CarlaDataProvider.request_new_actor(blueprint, transform)
            if actor is not None:
                self._batch1_actors.append(actor)
                self.other_actors.append(actor)
                controller = self._world.try_spawn_actor(controller_bp, carla.Transform(), actor)
                if controller is not None:
                    self._batch1_actors.append(controller)
                    self._batch1_walker_controllers.append(controller)

        for blueprint, transform in self._batch1_static_vehicle_spawns:
            actor = CarlaDataProvider.request_new_actor(blueprint, transform)
            if actor is not None:
                actor.set_simulate_physics(enabled=False)
                self._batch1_actors.append(actor)
                self.other_actors.append(actor)

        cone_bp = self._world.get_blueprint_library().find("static.prop.constructioncone")
        for transform in self._batch1_static_cone_spawns:
            actor = self._world.try_spawn_actor(cone_bp, transform)
            if actor is not None:
                self._batch1_actors.append(actor)
                self.other_actors.append(actor)

        for blueprint, transform in self._batch1_static_pedestrian_spawns:
            actor = CarlaDataProvider.request_new_actor(blueprint, transform)
            if actor is not None:
                self._batch1_actors.append(actor)
                self.other_actors.append(actor)

        # Final event actors — car, bicycle, pedestrian.
        actor = CarlaDataProvider.request_new_actor("vehicle.diamondback.century", self._event_bike_spawn)
        if actor is not None:
            actor.set_simulate_physics(enabled=True)
            self._event_bike = actor
            self.other_actors.append(actor)

        actor = CarlaDataProvider.request_new_actor("walker.pedestrian.0001", self._event_ped_spawn)
        if actor is not None:
            self._event_ped = actor
            self.other_actors.append(actor)

        # Accident scene barriers.
        barrier_bp = self._world.get_blueprint_library().find("static.prop.streetbarrier")
        for transform in self._accident_barrier_spawns:
            actor = self._world.try_spawn_actor(barrier_bp, transform)
            if actor is not None:
                self.other_actors.append(actor)

        # Construction barriers.
        if self._construction_barrier_spawns:
            for transform in self._construction_barrier_spawns:
                actor = self._world.try_spawn_actor(barrier_bp, transform)
                if actor is not None:
                    self.other_actors.append(actor)

        # Construction cones.
        if self._construction_cone_spawns:
            cone_bp = self._world.get_blueprint_library().find("static.prop.constructioncone")
            for transform in self._construction_cone_spawns:
                actor = self._world.try_spawn_actor(cone_bp, transform)
                if actor is not None:
                    self.other_actors.append(actor)

        # High cognitive load extras — parked cars (frozen) and extra cones.
    # end of _initialize_actors_impl

    # ------------------------------------------------------------------
    # Main behavior: freeze accident vehicles, then end at destination
    # ------------------------------------------------------------------

    def _create_behavior(self):
        """
        1. Wait 3 s for accident vehicles to settle, then freeze them.
        2. Wait for ego to reach the final destination.
        3. Clean up all actors.
        """
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

        main_sequence = py_trees.composites.Sequence("AccidentUpcomingMain")
        main_sequence.add_child(freeze_seq)
        main_sequence.add_child(end_condition)

        main_sequence.add_child(DespawnBatch(self._accident_scene_actors, name="CleanupAccidentScene"))
        main_sequence.add_child(DespawnBatch(self._batch1_actors, name="CleanupBatch1"))
        main_sequence.add_child(DespawnBatch(self._batch2_actors, name="CleanupBatch2"))
        main_sequence.add_child(DespawnBatch(self._batch3_actors, name="CleanupBatch3"))

        for actor in self.other_actors:
            if actor in self._accident_scene_actors:
                continue
            main_sequence.add_child(ActorDestroy(actor))

        return main_sequence

    # ------------------------------------------------------------------
    # Traffic flow helpers
    # ------------------------------------------------------------------

    def _flow_branch_triggered(self, flow, trigger_location, stop_location):
        """
        Sequence:
          1. Wait for ego to reach *trigger_location*.
          2. Run flow + stop-spawn in parallel (flow keeps running, spawning
             stops when ego reaches *stop_location*).
        Structure:
          Sequence:
            InTriggerDistanceToLocation (trigger)
            Parallel(SUCCESS_ON_ALL):
              flow
              Sequence: [InTriggerDist(stop) → StopFlowSpawning → WaitForever]
        """
        outer = py_trees.composites.Sequence("{}_Triggered".format(flow.name))
        outer.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0], trigger_location,
                distance=8.0,
                name="{}_StartTrigger".format(flow.name),
            )
        )

        par = py_trees.composites.Parallel(
            "{}_Branch".format(flow.name),
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
        )
        par.add_child(flow)
        stop_seq = py_trees.composites.Sequence("{}_StopSeq".format(flow.name))
        stop_seq.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0], stop_location,
                distance=20.0,
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
        outer.add_child(par)
        return outer

    # ------------------------------------------------------------------
    # Ambulance drive-away event
    # ------------------------------------------------------------------

    def _create_ambulance_event(self):
        """
        Waits for ego to reach the accident scene, then the event ambulance
        unfreezes, floors it toward the target, and switches to autopilot.
        Ends with WaitForever so the branch never causes an early scenario end.
        """
        seq = py_trees.composites.Sequence("AmbulanceEvent")
        seq.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0],
                self._ambulance_trigger_location,
                distance=10.0,
                name="AmbulanceTrigger",
            )
        )
        seq.add_child(
            VehicleDriveAway(
                self._event_ambulance,
                self._ambulance_target_location,
                CarlaDataProvider.get_traffic_manager_port(),
                name="AmbulanceDriveAway",
            )
        )
        seq.add_child(WaitForever())
        return seq

    # ------------------------------------------------------------------
    # Final event series — car, bicycle, pedestrian
    # ------------------------------------------------------------------

    def _create_event_car(self):
        """Car drives across ego's path when triggered."""
        seq = py_trees.composites.Sequence("EventCar")
        seq.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self._event_car_trigger,
            distance=10.0, name="EventCarTrigger",
        ))
        seq.add_child(VehicleFollowPath(
            self._event_car,
            self._event_car_path,
            CarlaDataProvider.get_traffic_manager_port(),
            initial_speed=0.5,
            throttle=0.35,
            name="EventCarDrive",
        ))
        seq.add_child(WaitForever())
        return seq

    def _create_event_bike(self):
        """Bicycle rides onto the road when triggered, then autopilots away."""
        seq = py_trees.composites.Sequence("EventBike")
        seq.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self._event_bike_trigger,
            distance=10.0, name="EventBikeTrigger",
        ))
        seq.add_child(VehicleDriveAway(
            self._event_bike,
            self._event_bike_target,
            CarlaDataProvider.get_traffic_manager_port(),
            initial_speed=0.0,
            name="EventBikeDrive",
        ))
        seq.add_child(WaitForever())
        return seq

    def _create_event_ped(self):
        """Pedestrian runs into the road in shock, then freezes."""
        seq = py_trees.composites.Sequence("EventPed")
        seq.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self._event_ped_trigger,
            distance=10.0, name="EventPedTrigger",
        ))
        seq.add_child(WalkerWalkTo(
            self._event_ped,
            self._event_ped_target,
            speed=5.0,
            name="EventPedRun",
        ))
        seq.add_child(WaitForever())
        return seq

    # ------------------------------------------------------------------
    # Bicycle near-miss branch
    # ------------------------------------------------------------------

    def _create_bike_behavior(self):
        """
        Waits for ego to reach the bike trigger location, then runs BikeNearMissEvent.
        BikeNearMissEvent spawns the bike and adjusts its speed so it arrives at
        _bike_sync_transform at the same time ego arrives at _bike_ego_sync_transform.
        Ends with WaitForever so the branch never causes an early scenario termination.
        """
        seq = py_trees.composites.Sequence("BikeNearMiss")
        seq.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0],
                self._bike_trigger_location,
                distance=10.0,
                name="BikeTrigger",
            )
        )
        seq.add_child(
            BikeNearMissEvent(
                self.ego_vehicles[0],
                self._bike_spawn_transform,
                self._bike_sync_transform,
                self._bike_end_transform,
                blueprint="vehicle.diamondback.century",
                ego_sync_transform=self._bike_ego_sync_transform,
                time_lead=0.2,   # bike crosses this many seconds before ego — tune if needed
                name="BikeNearMiss",
            )
        )
        seq.add_child(WaitForever())
        return seq

    # ------------------------------------------------------------------
    # Audio guidance branch
    # ------------------------------------------------------------------

    def _create_batch_despawn_behavior(self, actors, despawn_location, name):
        """
        Returns a Sequence that destroys *actors* once the ego comes within 20 m
        of *despawn_location*, then waits forever so it never triggers early SUCCESS.
        """
        seq = py_trees.composites.Sequence(name)
        seq.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0],
                despawn_location,
                distance=20.0,
                name="{}_Trigger".format(name),
            )
        )
        seq.add_child(DespawnBatch(actors, name="{}_Destroy".format(name)))
        seq.add_child(WaitForever())
        return seq

    def _create_triggered_vehicle_batch_behavior(self, vehicle_spawns, actors_out,
                                                 spawn_location, despawn_location,
                                                 tm_port, name):
        """
        Spawn a car-only batch when the ego reaches *spawn_location*, then
        destroy it once the ego reaches *despawn_location*.
        """
        seq = py_trees.composites.Sequence(name)
        seq.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0],
                spawn_location,
                distance=20.0,
                name="{}_SpawnTrigger".format(name),
            )
        )
        seq.add_child(SpawnActorGroup(
            vehicle_spawns=vehicle_spawns,
            walker_spawns=[],
            bicycle_spawns=[],
            actors_out=actors_out,
            tm_port=tm_port,
            name="{}_Spawn".format(name),
        ))
        seq.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0],
                despawn_location,
                distance=20.0,
                name="{}_DespawnTrigger".format(name),
            )
        )
        seq.add_child(DespawnBatch(actors_out, name="{}_Destroy".format(name)))
        seq.add_child(WaitForever())
        return seq

    def _create_triggered_mixed_batch_spawn_behavior(self, vehicle_spawns, walker_spawns,
                                                     bicycle_spawns, actors_out,
                                                     spawn_location, tm_port, name,
                                                     static_vehicle_spawns=None,
                                                     static_pedestrian_spawns=None):
        """
        Spawn a mixed batch when the ego reaches *spawn_location*.
        The batch persists until the scenario's end cleanup removes it.
        """
        from srunner.scenariomanager.scenarioatomics.atomic_behaviors import AtomicBehavior

        _static_vehicles = static_vehicle_spawns or []
        _static = static_pedestrian_spawns or []
        _aout = actors_out

        class _SpawnStaticVehicles(AtomicBehavior):
            def update(self_inner):
                for vehicle_spec in _static_vehicles:
                    if len(vehicle_spec) == 4:
                        blueprint, transform, light_state, simulate_physics = vehicle_spec
                    elif len(vehicle_spec) == 3:
                        blueprint, transform, light_state = vehicle_spec
                        simulate_physics = False
                    else:
                        blueprint, transform = vehicle_spec
                        light_state = None
                        simulate_physics = False
                    actor = None
                    z_offsets = [0.0, 0.5, 1.0, 1.5]
                    for z_offset in z_offsets:
                        spawn_transform = carla.Transform(
                            carla.Location(
                                x=transform.location.x,
                                y=transform.location.y,
                                z=transform.location.z + z_offset,
                            ),
                            transform.rotation,
                        )
                        actor = CarlaDataProvider.request_new_actor(blueprint, spawn_transform, tick=False)
                        if actor is not None:
                            if z_offset > 0.0:
                                print("[Batch3StaticVehicle] spawned {} using z offset +{:.2f}".format(
                                    blueprint, z_offset
                                ))
                            break
                    if actor is not None:
                        actor.set_simulate_physics(enabled=simulate_physics)
                        if light_state is not None:
                            actor.set_light_state(light_state)
                        _aout.append(actor)
                    else:
                        loc = transform.location
                        print("[Batch3StaticVehicle] failed to spawn {} at ({:.2f}, {:.2f}, {:.2f})".format(
                            blueprint, loc.x, loc.y, loc.z
                        ))
                return py_trees.common.Status.SUCCESS

        class _SpawnStaticPedestrians(AtomicBehavior):
            def update(self_inner):
                for blueprint, transform in _static:
                    actor = CarlaDataProvider.request_new_actor(blueprint, transform, tick=False)
                    if actor is not None:
                        _aout.append(actor)
                return py_trees.common.Status.SUCCESS

        seq = py_trees.composites.Sequence(name)
        seq.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0],
                spawn_location,
                distance=20.0,
                name="{}_SpawnTrigger".format(name),
            )
        )
        seq.add_child(SpawnActorGroup(
            vehicle_spawns=vehicle_spawns,
            walker_spawns=walker_spawns,
            bicycle_spawns=bicycle_spawns,
            actors_out=actors_out,
            tm_port=tm_port,
            name="{}_Spawn".format(name),
        ))
        if _static_vehicles:
            seq.add_child(_SpawnStaticVehicles("{}_StaticVehicles".format(name)))
        if _static:
            seq.add_child(_SpawnStaticPedestrians("{}_StaticPeds".format(name)))
        seq.add_child(WaitForever())
        return seq

    def _create_play_mp3_behavior(self):
        """
        Fires PlayMp3 cues sequentially as the ego approaches each navigation waypoint.
        Lives as a direct child of scenario_tree (root Parallel) and ends with
        WaitForever so it never triggers an early scenario termination.
        """
        play_mp3_root = py_trees.composites.Sequence("PlayMp3Branch")

        # Small drive-distance gate so cues don't fire before the ego moves.
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
          2. When ego reaches highway on-ramp → 90 km/h.
          3. When ego reaches highway exit → 40 km/h.
        Ends with WaitForever so the branch never causes an early scenario end.
        """
        seq = py_trees.composites.Sequence("SpeedZoneBranch")

        # Zone 1: highway — increase to 90 km/h
        seq.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            carla.Location(x=25.76, y=-174.01, z=0.20),
            distance=20.0,
            name="SpeedZone_90kmh_Trigger",
        ))
        seq.add_child(SetEgoMaxSpeed(
            self.ego_vehicles[0], 90.0, name="SpeedZone_90kmh",
        ))

        # Zone 2: back to urban — 40 km/h
        seq.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            carla.Location(x=14.96, y=-121.36, z=0.00),
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
        return []

    def __del__(self):
        if getattr(self, '_lsl_stream', None):
            self._lsl_stream.stop()
        self.remove_all_actors()
