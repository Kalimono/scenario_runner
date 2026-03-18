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
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ActorDestroy, Idle, WaitForever
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance,
    InTriggerDistanceToLocation,
)
from srunner.scenariomanager.scenarioatomics.atomic_behaviors_custom import EnableBatchAutopilot, FreezeActor, PlayMp3
from srunner.scenarios.basic_scenario import BasicScenario


# (location, label) pairs defining when each audio cue fires.
# The cue fires when the ego vehicle comes within CUE_TRIGGER_DISTANCE metres
# of the associated location.
_NAVIGATION_CUES = [
    (carla.Location(x=296.79, y=-168.97, z=0.20), "RIGHT"),
    (carla.Location(x=310.74, y=-141.19, z=0.00), "STRAIGHT"),
    (carla.Location(x=317.12, y=-69.15, z=0.00), "RIGHT ONTO HIGHWAY"),
    (carla.Location(x=299.49, y=13.42,   z=1.62), "ACCIDENT AHEAD"),
    (carla.Location(x=7.71,   y=-184.95, z=0.00), "RIGHT OFF HIGHWAY"),
    (carla.Location(x=180.30, y=-364.47, z=0.00), "ENTERING RESIDENTIAL AREA"),
    (carla.Location(x=202.50, y=-340.14, z=0.02), "STRAIGHT"),
    (carla.Location(x=201.34, y=-289.70, z=0.02), "LEFT"),
    (carla.Location(x=224.47, y=-246.00, z=0.00), "STRAIGHT"),
    (carla.Location(x=279.70, y=-246.38, z=0.00), "RIGHT"),
]

CUE_TRIGGER_DISTANCE = 10   # metres
TRAFFIC_BATCH_SEED = 42     # fixed seed for reproducible autopilot behaviour
DESPAWN_DISTANCE   = 20     # metres from despawn point before actors are removed


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
        # Vehicles get autopilot; pedestrians are placed statically.
        # All destroyed when the ego reaches the batch 1 despawn point.
        # ------------------------------------------------------------------
        self._batch1_vehicle_spawns = [
            ("vehicle.audi.tt", carla.Transform(
                carla.Location(x=349.06, y=-221.76, z=0.00),
                carla.Rotation(pitch=0.00, yaw=90.77,   roll=0.00),
            )),
            ("vehicle.chevrolet.impala", carla.Transform(
                carla.Location(x=333.91, y=-172.26, z=0.20),
                carla.Rotation(pitch=0.00, yaw=-179.67, roll=0.00),
            )),
            ("vehicle.nissan.patrol", carla.Transform(
                carla.Location(x=314.15, y=-131.51, z=0.00),
                carla.Rotation(pitch=0.00, yaw=270.51, roll=0.00),
            )),
            ("vehicle.seat.leon", carla.Transform(
                carla.Location(x=258.32, y=-134.64, z=0.02),
                carla.Rotation(pitch=0.00, yaw=-89.82,  roll=0.00),
            )),
            ("vehicle.mercedes.coupe", carla.Transform(
                carla.Location(x=217.17, y=-245.95, z=0.00),
                carla.Rotation(pitch=0.00, yaw=359.61,  roll=0.00),
            )),
            ("vehicle.toyota.prius", carla.Transform(
                carla.Location(x=361.25, y=-68.50,  z=0.00),
                carla.Rotation(pitch=0.00, yaw=179.27,  roll=0.00),
            )),
        ]
        self._batch1_pedestrian_spawns = [
            ("walker.pedestrian.0001", carla.Transform(
                carla.Location(x=307.55, y=-218.89, z=0.00),
                carla.Rotation(pitch=0.00, yaw=90.51,   roll=0.00),
            )),
            ("walker.pedestrian.0002", carla.Transform(
                carla.Location(x=306.97, y=-154.23, z=0.00),
                carla.Rotation(pitch=0.00, yaw=90.51,   roll=0.00),
            )),
            ("walker.pedestrian.0003", carla.Transform(
                carla.Location(x=262.26, y=-151.73, z=0.02),
                carla.Rotation(pitch=0.00, yaw=-89.82,  roll=0.00),
            )),
            ("walker.pedestrian.0004", carla.Transform(
                carla.Location(x=302.57, y=-176.32, z=0.20),
                carla.Rotation(pitch=0.00, yaw=180.33,  roll=0.00),
            )),
        ]
        self._batch1_despawn  = carla.Location(x=310.89, y=-79.25, z=0.00)
        self._batch1_actors   = []   # populated in _initialize_actors

        # ------------------------------------------------------------------
        # Batch 2 — highway + wider-area traffic, spawns when ego reaches
        # the batch 2 trigger point (end of the batch 1 zone).
        # ------------------------------------------------------------------
        _b2_blueprints = [
            "vehicle.audi.tt", "vehicle.chevrolet.impala", "vehicle.nissan.patrol",
            "vehicle.seat.leon", "vehicle.mercedes.coupe", "vehicle.toyota.prius",
            "vehicle.tesla.model3", "vehicle.audi.a2",
        ]
        _b2_transforms = [
            carla.Transform(carla.Location(x=381.55,  y=-127.02, z=0.00), carla.Rotation(pitch=0.00, yaw=90.60,   roll=0.00)),
            carla.Transform(carla.Location(x=385.21,  y=-142.36, z=0.00), carla.Rotation(pitch=0.00, yaw=90.60,   roll=0.00)),
            carla.Transform(carla.Location(x=392.46,  y=-158.05, z=0.00), carla.Rotation(pitch=0.00, yaw=-269.40, roll=0.00)),
            carla.Transform(carla.Location(x=389.49,  y=-216.86, z=0.00), carla.Rotation(pitch=0.00, yaw=90.60,   roll=0.00)),
            carla.Transform(carla.Location(x=394.30,  y=58.39,   z=0.00), carla.Rotation(pitch=0.00, yaw=-74.03,  roll=0.00)),
            carla.Transform(carla.Location(x=346.17,  y=42.05,   z=0.33), carla.Rotation(pitch=0.00, yaw=356.90,  roll=0.00)),
            carla.Transform(carla.Location(x=332.98,  y=31.50,   z=0.58), carla.Rotation(pitch=0.00, yaw=0.98,    roll=0.00)),
            carla.Transform(carla.Location(x=224.36,  y=36.64,   z=5.59), carla.Rotation(pitch=0.00, yaw=0.98,    roll=0.00)),
            carla.Transform(carla.Location(x=132.68,  y=28.08,   z=9.92), carla.Rotation(pitch=0.00, yaw=0.98,    roll=0.00)),
            carla.Transform(carla.Location(x=26.81,   y=27.19,   z=11.00),carla.Rotation(pitch=0.00, yaw=0.23,    roll=0.00)),
            carla.Transform(carla.Location(x=-56.35,  y=33.87,   z=10.03),carla.Rotation(pitch=0.00, yaw=0.08,    roll=0.00)),
            carla.Transform(carla.Location(x=-205.56, y=26.67,   z=4.44), carla.Rotation(pitch=0.00, yaw=0.08,    roll=0.00)),
            carla.Transform(carla.Location(x=-313.33, y=37.03,   z=0.44), carla.Rotation(pitch=0.00, yaw=0.08,    roll=0.00)),
            carla.Transform(carla.Location(x=-448.84, y=35.47,   z=0.00), carla.Rotation(pitch=0.00, yaw=-22.58,  roll=0.00)),
            carla.Transform(carla.Location(x=-482.35, y=117.23,  z=0.00), carla.Rotation(pitch=0.00, yaw=270.36,  roll=0.00)),
            carla.Transform(carla.Location(x=-494.24, y=226.68,  z=0.00), carla.Rotation(pitch=0.00, yaw=270.63,  roll=0.00)),
            carla.Transform(carla.Location(x=-457.19, y=329.98,  z=0.00), carla.Rotation(pitch=0.00, yaw=236.94,  roll=0.00)),
            carla.Transform(carla.Location(x=-398.27, y=396.34,  z=0.00), carla.Rotation(pitch=0.00, yaw=206.74,  roll=0.00)),
            carla.Transform(carla.Location(x=-245.97, y=404.24,  z=0.00), carla.Rotation(pitch=0.00, yaw=179.79,  roll=0.00)),
            carla.Transform(carla.Location(x=-146.07, y=403.81,  z=0.00), carla.Rotation(pitch=0.00, yaw=164.12,  roll=0.00)),
            carla.Transform(carla.Location(x=-19.10,  y=285.03,  z=0.00), carla.Rotation(pitch=0.00, yaw=109.70,  roll=0.00)),
            carla.Transform(carla.Location(x=-9.21,   y=-37.95,  z=0.00), carla.Rotation(pitch=0.00, yaw=89.78,   roll=0.00)),
            carla.Transform(carla.Location(x=-9.89,   y=-223.43, z=0.00), carla.Rotation(pitch=0.00, yaw=91.11,   roll=0.00)),
            carla.Transform(carla.Location(x=391.59,  y=-83.27,  z=0.00), carla.Rotation(pitch=0.00, yaw=90.60,   roll=0.00)),
            carla.Transform(carla.Location(x=384.38,  y=-56.49,  z=0.00), carla.Rotation(pitch=0.00, yaw=-269.56, roll=0.00)),
            carla.Transform(carla.Location(x=341.62,  y=21.14,   z=0.40), carla.Rotation(pitch=1.10, yaw=180.91,  roll=0.00)),
            carla.Transform(carla.Location(x=284.55,  y=13.17,   z=2.24), carla.Rotation(pitch=2.59, yaw=-179.02, roll=0.00)),
            carla.Transform(carla.Location(x=202.55,  y=15.27,   z=6.91), carla.Rotation(pitch=3.28, yaw=-179.02, roll=0.00)),
            carla.Transform(carla.Location(x=94.18,   y=9.97,    z=10.76),carla.Rotation(pitch=0.80, yaw=-539.77, roll=0.00)),
            carla.Transform(carla.Location(x=-26.40,  y=12.98,   z=10.55),carla.Rotation(pitch=-0.98,yaw=-179.77, roll=0.00)),
            carla.Transform(carla.Location(x=-96.61,  y=9.32,    z=9.35), carla.Rotation(pitch=-0.96,yaw=-539.92, roll=0.00)),
            carla.Transform(carla.Location(x=-158.87, y=5.73,    z=6.86), carla.Rotation(pitch=-2.97,yaw=-179.92, roll=0.00)),
            carla.Transform(carla.Location(x=-225.04, y=9.15,    z=3.43), carla.Rotation(pitch=-2.87,yaw=-179.92, roll=0.00)),
        ]
        self._batch2_vehicle_spawns = [
            (_b2_blueprints[i % len(_b2_blueprints)], t)
            for i, t in enumerate(_b2_transforms)
        ]
        self._batch2_trigger = carla.Location(x=310.28, y=-89.91, z=0.00)
        self._batch2_despawn = carla.Location(x=202.54, y=-341.75, z=0.02)
        self._batch2_actors  = []   # populated by SpawnBatch behavior at runtime

        # Scenario end location
        self._end_location = carla.Location(x=311.41, y=-216.00, z=0.00)

        super(AccidentAhead, self).__init__(
            "AccidentAhead",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

        # Attach the play_mp3 branch as its own child of the root scenario_tree
        # so it runs in parallel without interfering with main scenario logic.
        play_mp3_branch = self._create_play_mp3_behavior()
        self.scenario_tree.add_child(play_mp3_branch)
        play_mp3_branch.setup(timeout=1)

        # Batch 1 despawn branch — destroys ambient traffic when ego passes the despawn point.
        batch1_branch = self._create_batch_despawn_behavior(
            self._batch1_actors, self._batch1_despawn, "Batch1Despawn"
        )
        self.scenario_tree.add_child(batch1_branch)
        batch1_branch.setup(timeout=1)

        # Batch 2 autopilot trigger — activates when ego reaches the batch 2 trigger point.
        batch2_autopilot_branch = self._create_batch_autopilot_behavior(
            self._batch2_actors, self._batch2_trigger, "Batch2Autopilot"
        )
        self.scenario_tree.add_child(batch2_autopilot_branch)
        batch2_autopilot_branch.setup(timeout=1)

        # Batch 2 despawn branch.
        batch2_branch = self._create_batch_despawn_behavior(
            self._batch2_actors, self._batch2_despawn, "Batch2Despawn"
        )
        self.scenario_tree.add_child(batch2_branch)
        batch2_branch.setup(timeout=1)

        # Teleport spectator to overlook the accident scene at scenario start.
        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(
            carla.Location(x=291.08, y=-213.89, z=10.00),
            carla.Rotation(pitch=0.00, yaw=179.41, roll=0.00),
        ))

    # ------------------------------------------------------------------
    # Actor initialisation
    # ------------------------------------------------------------------

    def _initialize_actors(self, config):
        """Spawn accident-scene vehicles slightly above ground and let gravity settle them."""
        print("[AA] _initialize_actors START")
        _EMERGENCY_LIGHTS = carla.VehicleLightState(
            carla.VehicleLightState.Special1 | carla.VehicleLightState.Special2
        )

        print("[AA] Spawning accident scene actors ({})".format(len(self._accident_actor_spawns)))
        for blueprint, transform in self._accident_actor_spawns:
            print("[AA]   spawning {}".format(blueprint))
            actor = CarlaDataProvider.request_new_actor(blueprint, transform)
            if actor is not None:
                actor.set_simulate_physics(enabled=True)
                if "ambulance" in blueprint or "police" in blueprint or "firetruck" in blueprint:
                    actor.set_light_state(_EMERGENCY_LIGHTS)
                self.other_actors.append(actor)
                print("[AA]   -> OK id={}".format(actor.id))
            else:
                print("[AA]   -> FAILED to spawn {}".format(blueprint))

        print("[AA] Spawning accident scene pedestrians ({})".format(len(self._pedestrian_spawns)))
        for blueprint, transform in self._pedestrian_spawns:
            print("[AA]   spawning {}".format(blueprint))
            actor = CarlaDataProvider.request_new_actor(blueprint, transform)
            if actor is not None:
                self.other_actors.append(actor)
                print("[AA]   -> OK id={}".format(actor.id))
            else:
                print("[AA]   -> FAILED")

        # Batch 1 ambient traffic — vehicles on autopilot, pedestrians static.
        tm_port = CarlaDataProvider.get_traffic_manager_port()
        tm = CarlaDataProvider.get_client().get_trafficmanager(tm_port)
        tm.set_random_device_seed(TRAFFIC_BATCH_SEED)

        print("[AA] Spawning batch 1 vehicles ({})".format(len(self._batch1_vehicle_spawns)))
        for blueprint, transform in self._batch1_vehicle_spawns:
            print("[AA]   spawning {} at ({:.1f},{:.1f})".format(blueprint, transform.location.x, transform.location.y))
            actor = CarlaDataProvider.request_new_actor(blueprint, transform)
            if actor is not None:
                actor.set_autopilot(True, tm_port)
                self._batch1_actors.append(actor)
                print("[AA]   -> OK id={}".format(actor.id))
            else:
                print("[AA]   -> FAILED")

        print("[AA] Spawning batch 1 pedestrians ({})".format(len(self._batch1_pedestrian_spawns)))
        for blueprint, transform in self._batch1_pedestrian_spawns:
            print("[AA]   spawning {}".format(blueprint))
            walker = CarlaDataProvider.request_new_actor(blueprint, transform)
            if walker is not None:
                yaw_rad = math.radians(transform.rotation.yaw)
                walker.apply_control(carla.WalkerControl(
                    direction=carla.Vector3D(math.cos(yaw_rad), math.sin(yaw_rad), 0.0),
                    speed=1.4,
                    jump=False,
                ))
                self._batch1_actors.append(walker)
                print("[AA]   -> OK id={}".format(walker.id))
            else:
                print("[AA]   -> FAILED")

        print("[AA] Spawning batch 2 vehicles ({})".format(len(self._batch2_vehicle_spawns)))
        for i, (blueprint, transform) in enumerate(self._batch2_vehicle_spawns):
            print("[AA]   [{}/{}] spawning {} at ({:.1f},{:.1f},{:.1f})".format(
                i + 1, len(self._batch2_vehicle_spawns), blueprint,
                transform.location.x, transform.location.y, transform.location.z))
            actor = CarlaDataProvider.request_new_actor(blueprint, transform)
            if actor is not None:
                self._batch2_actors.append(actor)
                print("[AA]   -> OK id={}".format(actor.id))
            else:
                print("[AA]   -> FAILED (spawn rejected)")

        print("[AA] _initialize_actors DONE — other_actors={} batch1={} batch2={}".format(
            len(self.other_actors), len(self._batch1_actors), len(self._batch2_actors)))

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
        # Wait 3 s for accident vehicles to fall and settle, then freeze them in place.
        freeze_seq = py_trees.composites.Sequence("FreezeAccidentScene")
        freeze_seq.add_child(Idle(3.0, name="WaitForSettling"))
        for actor in self.other_actors:
            freeze_seq.add_child(FreezeActor(actor, name="FreezeActor_{}".format(actor.id)))

        # Gate the end condition behind the last batch despawn point (currently batch 2).
        passed_last_batch = InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            self._batch2_despawn,
            distance=DESPAWN_DISTANCE,
            name="PassedLastBatch",
        )

        end_condition = InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            self._end_location,
            distance=5.0,
            name="ReachedDestination",
        )

        root = py_trees.composites.Sequence("AccidentAhead")
        root.add_child(freeze_seq)
        root.add_child(passed_last_batch)
        root.add_child(end_condition)

        # Clean up accident-scene actors on completion
        for actor in self.other_actors:
            root.add_child(ActorDestroy(actor))

        return root

    # ------------------------------------------------------------------
    # Batch traffic despawn branch (own root-level branch)
    # ------------------------------------------------------------------

    def _create_batch_autopilot_behavior(self, actors, trigger_location, name):
        """
        Returns a Sequence that waits for the ego to reach *trigger_location*,
        then enables autopilot on all *actors*, then waits forever.
        """
        tm_port = CarlaDataProvider.get_traffic_manager_port()
        seq = py_trees.composites.Sequence(name)
        seq.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0], trigger_location,
                distance=DESPAWN_DISTANCE, name="{}_Trigger".format(name),
            )
        )
        seq.add_child(EnableBatchAutopilot(actors, tm_port, name="{}_Enable".format(name)))
        seq.add_child(WaitForever())
        return seq

    def _create_batch_spawn_despawn_behavior(self, spawns, actors_out,
                                              trigger_location, despawn_location, name):
        """
        Returns a Sequence that:
          1. Waits until the ego reaches *trigger_location*.
          2. Spawns all vehicles in *spawns* with autopilot (SpawnBatch).
          3. Waits until the ego reaches *despawn_location*.
          4. Destroys all spawned actors (DespawnBatch).
          5. Waits forever so it cannot trigger an early scenario end.
        """
        tm_port = CarlaDataProvider.get_traffic_manager_port()
        seq = py_trees.composites.Sequence(name)
        seq.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0], trigger_location,
                distance=DESPAWN_DISTANCE, name="{}_SpawnTrigger".format(name),
            )
        )
        seq.add_child(SpawnBatch(spawns, actors_out, tm_port, name="{}_Spawn".format(name)))
        seq.add_child(
            InTriggerDistanceToLocation(
                self.ego_vehicles[0], despawn_location,
                distance=DESPAWN_DISTANCE, name="{}_DespawnTrigger".format(name),
            )
        )
        seq.add_child(DespawnBatch(actors_out, name="{}_Despawn".format(name)))
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
        for actor in actors:
            seq.add_child(ActorDestroy(actor, name="Destroy_{}".format(actor.id)))
        seq.add_child(WaitForever())
        return seq

    # ------------------------------------------------------------------
    # Play-MP3 guidance branch (own root-level branch)
    # ------------------------------------------------------------------

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
            cue.add_child(PlayMp3(label))
            play_mp3_root.add_child(cue)

        # Keep the branch alive forever so it cannot stop the scenario_tree
        play_mp3_root.add_child(WaitForever())

        return play_mp3_root

    # ------------------------------------------------------------------
    # Criteria
    # ------------------------------------------------------------------

    def _create_test_criteria(self):
        criteria = []
        criteria.append(CollisionTest(self.ego_vehicles[0]))
        return criteria

    def __del__(self):
        self.remove_all_actors()
