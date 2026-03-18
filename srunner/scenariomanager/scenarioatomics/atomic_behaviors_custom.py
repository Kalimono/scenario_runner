#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Custom atomic behaviors for EA scenario guidance.

This module provides placeholder behaviors that can be extended with real
audio playback functionality.
"""

import os
import threading

import carla
import py_trees

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import AtomicBehavior

# Folder that contains the MP3 guidance files.
# atomic_behaviors_custom.py lives at srunner/scenariomanager/scenarioatomics/
_ASSETS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "scenarios", "assets",
)

try:
    import pygame
    pygame.mixer.init()
    _PYGAME_AVAILABLE = True
except Exception:
    _PYGAME_AVAILABLE = False


def _play_mp3_async(path):
    """Play an MP3 file in a background thread so the behavior tree is not blocked."""
    def _play():
        try:
            pygame.mixer.music.load(path)
            pygame.mixer.music.play()
        except Exception as exc:
            print("[PlayMp3] Playback error: {}".format(exc))
    threading.Thread(target=_play, daemon=True).start()


class PlayMp3(AtomicBehavior):
    """
    Placeholder behavior representing an MP3 audio cue played to guide the driver.

    This is a no-op stub. Replace the update() body with real audio playback
    (e.g. pygame.mixer or a subprocess call) when MP3 files are available.

    Args:
        label (str): The navigation instruction label, e.g. "RIGHT", "STRAIGHT",
                     "ACCIDENT AHEAD". Used for logging and as a hook for the
                     real implementation to select the correct audio file.
        name (str): Name of the behavior node shown in the py_trees visualisation.
    """

    def __init__(self, label, name="PlayMp3"):
        super(PlayMp3, self).__init__("{}: {}".format(name, label))
        self.logger.debug("%s.__init__()" % self.__class__.__name__)
        self._label = label

    def update(self):
        self.logger.debug("%s.update() — cue: '%s'" % (self.__class__.__name__, self._label))
        filename = self._label.lower().replace(" ", "_") + ".mp3"
        path = os.path.join(_ASSETS_DIR, filename)
        if _PYGAME_AVAILABLE and os.path.isfile(path):
            _play_mp3_async(path)
        else:
            print("[PlayMp3] {} (no audio file: {})".format(self._label, filename))
        return py_trees.common.Status.SUCCESS


class StartWalkerControllers(AtomicBehavior):
    """
    Starts a list of controller.ai.walker actors on the first behavior-tree tick
    (by which point the world has ticked at least once and the controllers are live).

    Each controller is sent to a random navigation location at a normal walking speed.
    Returns SUCCESS immediately so it does not block the sequence.

    Args:
        world: The carla.World instance.
        controllers (list): carla.Actor objects of type controller.ai.walker.
        speed (float): Walk speed in m/s (default 1.4 ≈ 5 km/h).
    """

    def __init__(self, world, controllers, speed=1.4, name="StartWalkerControllers"):
        super(StartWalkerControllers, self).__init__(name)
        self._world = world
        self._controllers = controllers
        self._speed = speed

    def update(self):
        for controller in self._controllers:
            controller.start()
            controller.go_to_location(self._world.get_random_location_from_navigation())
            controller.set_max_speed(self._speed)
        return py_trees.common.Status.SUCCESS


class SpawnBatch(AtomicBehavior):
    """
    Spawns a list of vehicles with autopilot on the first behavior-tree tick.
    Appends successfully spawned actors to *actors_out* so a later DespawnBatch
    can destroy them without needing to know the IDs at tree-build time.

    Args:
        spawns (list): List of (blueprint_str, carla.Transform) pairs.
        actors_out (list): Shared list that receives the spawned carla.Actor objects.
        tm_port (int): Traffic manager port for autopilot.
    """

    def __init__(self, spawns, actors_out, tm_port, name="SpawnBatch"):
        super(SpawnBatch, self).__init__(name)
        self._spawns = spawns
        self._actors_out = actors_out
        self._tm_port = tm_port

    def initialise(self):
        self._phase = 0

    def update(self):
        from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
        if self._phase == 0:
            # Phase 1: spawn all actors without ticking so the scenario manager's
            # sync loop is not disturbed. Actors are registered but not yet live.
            for blueprint, transform in self._spawns:
                actor = CarlaDataProvider.request_new_actor(blueprint, transform, tick=False)
                if actor is not None:
                    self._actors_out.append(actor)
            self._phase = 1
            return py_trees.common.Status.RUNNING  # let the world tick once
        else:
            # Phase 2: actors are live — safe to enable autopilot.
            for actor in self._actors_out:
                actor.set_autopilot(True, self._tm_port)
            return py_trees.common.Status.SUCCESS


class DespawnBatch(AtomicBehavior):
    """
    Destroys all actors in *actors* and clears the list.

    Args:
        actors (list): Shared list of carla.Actor objects to destroy.
    """

    def __init__(self, actors, name="DespawnBatch"):
        super(DespawnBatch, self).__init__(name)
        self._actors = actors

    def update(self):
        for actor in list(self._actors):
            if actor.is_alive:
                actor.destroy()
        self._actors.clear()
        return py_trees.common.Status.SUCCESS


class EnableBatchAutopilot(AtomicBehavior):
    """
    Enables autopilot on a list of pre-spawned vehicles that were spawned
    without autopilot so they stay stationary until the ego reaches a trigger.

    Args:
        actors (list): carla.Vehicle objects to activate.
        tm_port (int): Traffic manager port for autopilot.
    """

    def __init__(self, actors, tm_port, name="EnableBatchAutopilot"):
        super(EnableBatchAutopilot, self).__init__(name)
        self._actors = actors
        self._tm_port = tm_port

    def update(self):
        print("[EnableBatchAutopilot] Enabling autopilot on {} actors".format(len(self._actors)))
        for actor in self._actors:
            print("[EnableBatchAutopilot]   actor id={} alive={}".format(actor.id, actor.is_alive))
            actor.set_autopilot(True, self._tm_port)
        print("[EnableBatchAutopilot] Done")
        return py_trees.common.Status.SUCCESS


class FreezeActor(AtomicBehavior):
    """
    Disables physics simulation on an actor and zeroes its velocity so it
    stays exactly where it landed after being dropped under gravity.

    Args:
        actor: The carla.Actor to freeze.
        name (str): Node name shown in the py_trees visualisation.
    """

    def __init__(self, actor, name="FreezeActor"):
        super(FreezeActor, self).__init__(name)
        self._actor = actor

    def update(self):
        print("[FreezeActor] Freezing actor id={}".format(self._actor.id))
        self._actor.set_target_velocity(carla.Vector3D(0, 0, 0))
        self._actor.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        self._actor.set_simulate_physics(enabled=False)
        print("[FreezeActor] Done id={}".format(self._actor.id))
        return py_trees.common.Status.SUCCESS
