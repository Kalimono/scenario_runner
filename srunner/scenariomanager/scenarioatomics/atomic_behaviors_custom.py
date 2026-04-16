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

import math
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

    def __init__(self, world, controllers, speed=1.4, destination_mode="random_nav",
                 name="StartWalkerControllers"):
        super(StartWalkerControllers, self).__init__(name)
        self._world = world
        self._controllers = controllers
        self._speed = speed
        self._destination_mode = destination_mode

    def _far_nav_destination(self, origin, min_distance=50.0, attempts=20):
        fallback = None
        for _ in range(attempts):
            dest = self._world.get_random_location_from_navigation()
            if dest is None:
                continue
            fallback = dest
            if origin is None or dest.distance(origin) >= min_distance:
                return dest
        return fallback

    def update(self):
        for controller in self._controllers:
            controller.start()
            walker = getattr(controller, "parent", None)
            origin = walker.get_location() if walker is not None and walker.is_alive else None
            if self._destination_mode == "far_nav":
                destination = self._far_nav_destination(origin)
            else:
                destination = self._world.get_random_location_from_navigation()
            if destination is not None:
                controller.go_to_location(destination)
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


class KeepTrafficLightsGreen(AtomicBehavior):
    """Keep traffic-light cycles short so waits do not dominate the scenario."""

    def __init__(self, name="KeepTrafficLightsGreen"):
        super(KeepTrafficLightsGreen, self).__init__(name)
        self._configured = False

    def update(self):
        from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
        world = CarlaDataProvider.get_world()
        if world is None:
            return py_trees.common.Status.RUNNING

        if not self._configured:
            try:
                world.freeze_all_traffic_lights(False)
            except RuntimeError:
                pass
            self._configured = True

        for traffic_light in world.get_actors().filter("traffic.traffic_light*"):
            try:
                traffic_light.freeze(False)
                traffic_light.set_red_time(2.0)
                traffic_light.set_yellow_time(1.0)
                traffic_light.set_green_time(6.0)
            except RuntimeError:
                pass
        return py_trees.common.Status.RUNNING


class LogNavigationCue(AtomicBehavior):
    """Print a line when a navigation cue activates."""

    def __init__(self, label, location, name="LogNavigationCue"):
        super(LogNavigationCue, self).__init__(name)
        self._label = label
        self._location = location

    def update(self):
        print("[NavigationCue] ACTIVATED '{}' at ({:.2f}, {:.2f}, {:.2f})".format(
            self._label, self._location.x, self._location.y, self._location.z
        ))
        return py_trees.common.Status.SUCCESS


class NavigationMarker(AtomicBehavior):
    """Shows one pink navigation marker until the ego reaches it."""

    _MARKER_COLOR = carla.Color(255, 0, 255)
    _LABEL_COLOR = carla.Color(255, 80, 255)

    def __init__(self, ego_actor, location, label, trigger_distance=15.0, name="NavigationMarker"):
        super(NavigationMarker, self).__init__("{}: {}".format(name, label))
        self._ego_actor = ego_actor
        self._location = location
        self._label = label
        self._trigger_distance = trigger_distance
        self._drawn = False

    def initialise(self):
        if self._drawn:
            return
        from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
        world = CarlaDataProvider.get_world()
        if world is None:
            return
        world.debug.draw_point(
            carla.Location(x=self._location.x, y=self._location.y, z=self._location.z + 1.5),
            size=0.55,
            color=self._MARKER_COLOR,
            life_time=9999,
            persistent_lines=True,
        )
        world.debug.draw_string(
            carla.Location(x=self._location.x, y=self._location.y, z=self._location.z + 3.0),
            self._label,
            draw_shadow=True,
            color=self._LABEL_COLOR,
            life_time=9999,
            persistent_lines=True,
        )
        self._drawn = True

    def update(self):
        from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
        ego_location = CarlaDataProvider.get_location(self._ego_actor)
        if ego_location is None:
            return py_trees.common.Status.RUNNING
        if ego_location.distance(self._location) <= self._trigger_distance:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class SetEgoMaxSpeed(AtomicBehavior):
    """Set ego max speed in km/h if the vehicle wrapper supports it."""

    def __init__(self, ego_actor, speed_kmh, name="SetEgoMaxSpeed"):
        super(SetEgoMaxSpeed, self).__init__(name)
        self._ego_actor = ego_actor
        self._speed_kmh = speed_kmh

    def update(self):
        if hasattr(self._ego_actor, "set_max_speed"):
            self._ego_actor.set_max_speed(self._speed_kmh)
        print("[SetEgoMaxSpeed] Ego max speed set to {:.1f} km/h".format(self._speed_kmh))
        return py_trees.common.Status.SUCCESS


class StopFlowSpawning(AtomicBehavior):
    """Stop an actor-flow object from spawning more actors."""

    def __init__(self, flow, ego_actor=None, stop_location=None, name="StopFlowSpawning"):
        super(StopFlowSpawning, self).__init__(name)
        self._flow = flow

    def update(self):
        if hasattr(self._flow, "stop_spawning"):
            self._flow.stop_spawning()
        elif hasattr(self._flow, "_spawn_dist"):
            self._flow._spawn_dist = float("inf")
        return py_trees.common.Status.SUCCESS


class WalkerWalkTo(AtomicBehavior):
    """Move a walker actor toward a target location using walker controls."""

    def __init__(self, walker, target_location, speed=1.4, name="WalkerWalkTo"):
        super(WalkerWalkTo, self).__init__(name)
        self._walker = walker
        self._target = target_location
        self._speed = speed

    def update(self):
        if self._walker is None or not getattr(self._walker, "is_alive", False):
            return py_trees.common.Status.SUCCESS
        try:
            loc = self._walker.get_location()
            delta = self._target - loc
            distance = (delta.x ** 2 + delta.y ** 2 + delta.z ** 2) ** 0.5
            if distance < 0.5:
                self._walker.apply_control(carla.WalkerControl(speed=0.0))
                return py_trees.common.Status.SUCCESS
            direction = carla.Vector3D(delta.x / distance, delta.y / distance, 0.0)
            self._walker.apply_control(carla.WalkerControl(direction=direction, speed=self._speed, jump=False))
        except RuntimeError:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class SpawnActorGroup(AtomicBehavior):
    """Spawn vehicles, bicycles, and walkers as one triggered group."""

    def __init__(self, vehicle_spawns, walker_spawns, bicycle_spawns,
                 actors_out, tm_port, walker_controllers_out=None,
                 name="SpawnActorGroup"):
        super(SpawnActorGroup, self).__init__(name)
        self._vehicle_spawns = vehicle_spawns
        self._walker_spawns = walker_spawns
        self._bicycle_spawns = bicycle_spawns
        self._actors_out = actors_out
        self._tm_port = tm_port
        self._walker_controllers_out = walker_controllers_out
        self._phase = 0
        self._vehicles = []
        self._controllers = []

    def initialise(self):
        self._phase = 0
        self._vehicles = []
        self._controllers = []

    def update(self):
        from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
        world = CarlaDataProvider.get_world()
        if self._phase == 0:
            controller_bp = world.get_blueprint_library().find("controller.ai.walker")
            for blueprint, transform in self._vehicle_spawns + self._bicycle_spawns:
                actor = CarlaDataProvider.request_new_actor(blueprint, transform, rolename="autopilot", tick=False)
                if actor is not None:
                    self._vehicles.append(actor)
                    self._actors_out.append(actor)
            for blueprint, transform in self._walker_spawns:
                walker = CarlaDataProvider.request_new_actor(blueprint, transform, tick=False)
                if walker is not None:
                    self._actors_out.append(walker)
                    controller = world.try_spawn_actor(controller_bp, carla.Transform(), walker)
                    if controller is not None:
                        self._controllers.append(controller)
                        self._actors_out.append(controller)
                        if self._walker_controllers_out is not None:
                            self._walker_controllers_out.append(controller)
            self._phase = 1
            return py_trees.common.Status.RUNNING

        if self._phase == 1:
            for vehicle in self._vehicles:
                if getattr(vehicle, "is_alive", False):
                    vehicle.set_autopilot(True, self._tm_port)
            self._phase = 2
            return py_trees.common.Status.RUNNING

        for controller in self._controllers:
            if getattr(controller, "is_alive", False):
                controller.start()
                dest = world.get_random_location_from_navigation()
                if dest is not None:
                    controller.go_to_location(dest)
                controller.set_max_speed(1.4)
        return py_trees.common.Status.SUCCESS


class VehicleFollowPath(AtomicBehavior):
    """Drive a vehicle toward a list of target locations with simple controls."""

    def __init__(self, actor, path, tm_port, initial_speed=0.0, throttle=0.5, name="VehicleFollowPath"):
        super(VehicleFollowPath, self).__init__(name)
        self._actor = actor
        self._path = path
        self._tm_port = tm_port
        self._initial_speed = initial_speed
        self._throttle = throttle
        self._index = 0
        self._started = False

    def update(self):
        if self._actor is None or not getattr(self._actor, "is_alive", False):
            return py_trees.common.Status.FAILURE
        if not self._started:
            self._actor.set_simulate_physics(True)
            self._started = True
        if self._index >= len(self._path):
            self._actor.set_autopilot(True, self._tm_port)
            return py_trees.common.Status.SUCCESS
        target = self._path[self._index]
        loc = self._actor.get_location()
        dx = target.x - loc.x
        dy = target.y - loc.y
        distance = (dx ** 2 + dy ** 2) ** 0.5
        if distance < 3.0:
            self._index += 1
            return py_trees.common.Status.RUNNING
        yaw = math.atan2(dy, dx)
        current_yaw = math.radians(self._actor.get_transform().rotation.yaw)
        steer = max(-1.0, min(1.0, (yaw - current_yaw + math.pi) % (2 * math.pi) - math.pi))
        self._actor.apply_control(carla.VehicleControl(throttle=self._throttle, steer=steer))
        return py_trees.common.Status.RUNNING


class VehicleDriveAway(VehicleFollowPath):
    """Drive a vehicle to one target and then hand it to autopilot."""

    def __init__(self, actor, target_location, tm_port, initial_speed=0.0, name="VehicleDriveAway"):
        super(VehicleDriveAway, self).__init__(
            actor, [target_location], tm_port, initial_speed=initial_speed, throttle=1.0, name=name
        )


class AmbulanceResponse(AtomicBehavior):
    """Spawn an ambulance, launch it briefly, then hand it to Traffic Manager."""

    _EMERGENCY_LIGHTS = carla.VehicleLightState(
        carla.VehicleLightState.Special1 | carla.VehicleLightState.Special2
    )

    def __init__(self, spawn_transform, despawn_location, tm_port,
                 initial_speed=25.0, velocity_duration=0.5, despawn_distance=30.0,
                 name="AmbulanceResponse"):
        super(AmbulanceResponse, self).__init__(name)
        self._spawn_transform = spawn_transform
        self._despawn_location = despawn_location
        self._tm_port = tm_port
        self._initial_speed = initial_speed
        self._velocity_duration = velocity_duration
        self._despawn_distance = despawn_distance
        self._actor = None
        self._start_time = None
        self._state = 0

    def update(self):
        import time
        from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

        if self._state == 0:
            self._actor = CarlaDataProvider.request_new_actor(
                "vehicle.ford.ambulance", self._spawn_transform, tick=False
            )
            if self._actor is None:
                return py_trees.common.Status.FAILURE
            self._actor.set_simulate_physics(True)
            self._actor.set_light_state(self._EMERGENCY_LIGHTS)
            forward = self._spawn_transform.get_forward_vector()
            self._actor.enable_constant_velocity(carla.Vector3D(
                forward.x * self._initial_speed,
                forward.y * self._initial_speed,
                forward.z * self._initial_speed,
            ))
            self._start_time = time.time()
            self._state = 1
            return py_trees.common.Status.RUNNING

        if self._state == 1:
            if time.time() - self._start_time < self._velocity_duration:
                return py_trees.common.Status.RUNNING
            try:
                self._actor.disable_constant_velocity()
            except RuntimeError:
                pass
            if getattr(self._actor, "is_alive", False):
                self._actor.set_autopilot(True, self._tm_port)
            self._state = 2
            return py_trees.common.Status.RUNNING

        if self._actor is None or not getattr(self._actor, "is_alive", False):
            return py_trees.common.Status.SUCCESS
        if self._actor.get_location().distance(self._despawn_location) <= self._despawn_distance:
            self._actor.destroy()
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class BikeNearMissEvent(AtomicBehavior):
    """Spawn and move a bike along a short path toward a target point."""

    def __init__(self, ego_actor, spawn_transform, sync_transform, end_transform,
                 blueprint="vehicle.diamondback.century", ego_sync_transform=None,
                 time_lead=0.2, min_speed=3.0, max_speed=10.0,
                 fixed_speed=None, path_transforms=None, name="BikeNearMissEvent"):
        super(BikeNearMissEvent, self).__init__(name)
        self._ego_actor = ego_actor
        self._spawn_transform = spawn_transform
        self._blueprint = blueprint
        self._ego_sync_transform = ego_sync_transform
        self._time_lead = time_lead
        self._min_speed = min_speed
        self._max_speed = max_speed
        self._fixed_speed = fixed_speed
        self._path = [t.location for t in (path_transforms or [sync_transform, end_transform])]
        self._actor = None
        self._index = 0
        self._last_update = None

    def update(self):
        import time
        from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
        if self._actor is None:
            self._actor = CarlaDataProvider.request_new_actor(self._blueprint, self._spawn_transform, tick=False)
            if self._actor is None:
                return py_trees.common.Status.FAILURE
            self._actor.set_simulate_physics(False)
            self._last_update = time.time()
            return py_trees.common.Status.RUNNING

        now = time.time()
        dt = max(0.05, now - (self._last_update or now))
        self._last_update = now
        if self._index >= len(self._path):
            return py_trees.common.Status.SUCCESS

        target = self._path[self._index]
        transform = self._actor.get_transform()
        loc = transform.location
        dx = target.x - loc.x
        dy = target.y - loc.y
        dz = target.z - loc.z
        distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
        if distance < 0.5:
            self._index += 1
            return py_trees.common.Status.RUNNING

        speed = self._fixed_speed if self._fixed_speed is not None else self._min_speed
        if self._ego_sync_transform is not None and self._fixed_speed is None:
            ego_loc = CarlaDataProvider.get_location(self._ego_actor)
            if ego_loc is not None:
                ego_speed = self._ego_actor.get_velocity()
                ego_speed_mag = max(0.1, (ego_speed.x ** 2 + ego_speed.y ** 2 + ego_speed.z ** 2) ** 0.5)
                ego_dist = ego_loc.distance(self._ego_sync_transform.location)
                target_time = max(0.1, ego_dist / ego_speed_mag - self._time_lead)
                speed = max(self._min_speed, min(self._max_speed, distance / target_time))
                if ego_loc.distance(self._ego_sync_transform.location) <= 3.0:
                    speed = max(speed, 8.0)

        step = min(distance, speed * dt)
        next_loc = carla.Location(
            x=loc.x + dx / distance * step,
            y=loc.y + dy / distance * step,
            z=loc.z + dz / distance * step,
        )
        yaw = math.degrees(math.atan2(dy, dx))
        self._actor.set_transform(carla.Transform(next_loc, carla.Rotation(yaw=yaw)))
        return py_trees.common.Status.RUNNING
