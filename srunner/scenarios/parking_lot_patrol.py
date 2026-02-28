#!/usr/bin/env python

# Copyright (c) 2024
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
ParkingLotPatrol scenario:

Ego vehicle spawns in an urban parking lot.
Three NPC vehicles drive on Traffic Manager autopilot.
Two pedestrians patrol sidewalk loops around nearby buildings.

When the ego vehicle enters within PED_TRIGGER_DISTANCE metres of a pedestrian,
that pedestrian stops patrolling and begins crossing the road.  It walks toward
the closest point on the ego's current driving lane, and once it reaches that
point it continues walking blindly in the same direction.

Pedestrians are controlled via carla.WalkerControl (direct velocity commands)
rather than the AI walker controller, which avoids navigation-mesh coverage
issues in parking lot areas.

Scenario end conditions:
  - Ego collides with any vehicle or pedestrian  (CollisionTest, terminate_on_failure)
  - Ego enters within END_PROXIMITY_DISTANCE metres of any configured end location
    (add carla.Location entries to _END_LOCATIONS to activate this condition)
"""

import math

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import AtomicBehavior
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToLocation
from srunner.scenarios.basic_scenario import BasicScenario


# ---------------------------------------------------------------------------
# Custom atomic behavior – controls a single pedestrian via WalkerControl
# ---------------------------------------------------------------------------

class PedestrianBehavior(AtomicBehavior):
    """
    Drives a pedestrian through two phases using carla.WalkerControl
    (direct velocity commands, no nav-mesh dependency).

    PATROLLING
        Loops through an ordered list of sidewalk transforms, walking
        toward each in sequence by computing a direction vector each tick.

    CROSSING
        Triggered when the ego enters within ``trigger_dist`` metres.
        The pedestrian walks toward the closest point on the ego's driving
        lane.  Once that point is reached it continues in the same direction.

    The behavior always returns RUNNING; termination is handled elsewhere.

    Parameters
    ----------
    walker       : carla.Walker             – the pedestrian actor
    ego          : carla.Actor              – ego vehicle
    patrol_wps   : list[carla.Transform]    – ordered patrol waypoints (looped)
    trigger_dist : float  – metres; ego closer than this switches to CROSSING
    walk_speed   : float  – m/s
    reach_dist   : float  – metres; within this a waypoint is "reached"
    """

    _PATROLLING = 0
    _CROSSING = 1

    def __init__(self, walker, ego, patrol_wps,
                 trigger_dist=20.0, walk_speed=1.4, reach_dist=2.0,
                 name="PedestrianBehavior"):
        super().__init__(name)
        self._walker = walker
        self._ego = ego
        self._patrol_wps = patrol_wps
        self._trigger_dist = trigger_dist
        self._walk_speed = walk_speed
        self._reach_dist = reach_dist

        self._state = self._PATROLLING
        # Start at index 1 so the first go_to is an actual move, not a zero-distance no-op
        self._patrol_index = 1 % len(patrol_wps)

        self._cross_target = None   # carla.Location – target on ego's road
        self._cross_dir = None      # (dx, dy) unit vector for the "continue" phase
        self._crossed = False       # True once the road waypoint is reached

        self._map = CarlaDataProvider.get_map()

    def update(self):
        walker_loc = self._walker.get_location()
        ego_loc = self._ego.get_location()

        if walker_loc is None or ego_loc is None:
            return py_trees.common.Status.RUNNING

        if self._state == self._PATROLLING:
            # Switch to crossing when ego is close enough
            if walker_loc.distance(ego_loc) < self._trigger_dist:
                self._start_crossing(walker_loc, ego_loc)
                return py_trees.common.Status.RUNNING

            # Advance patrol when current waypoint is reached
            target_loc = self._patrol_wps[self._patrol_index].location
            if walker_loc.distance(target_loc) < self._reach_dist:
                self._patrol_index = (self._patrol_index + 1) % len(self._patrol_wps)
                target_loc = self._patrol_wps[self._patrol_index].location

            self._apply_control(walker_loc, target_loc)

        elif self._state == self._CROSSING:
            if self._cross_dir is not None:
                if not self._crossed and self._cross_target is not None:
                    if walker_loc.distance(self._cross_target) < self._reach_dist:
                        self._crossed = True

                if self._crossed:
                    # Continue straight ahead – target 5 m in the stored direction
                    dx, dy = self._cross_dir
                    target = carla.Location(
                        x=walker_loc.x + dx * 5.0,
                        y=walker_loc.y + dy * 5.0,
                        z=walker_loc.z
                    )
                else:
                    target = self._cross_target

                self._apply_control(walker_loc, target)

        return py_trees.common.Status.RUNNING

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _apply_control(self, from_loc, to_loc):
        """Compute a direction vector and apply WalkerControl."""
        dx = to_loc.x - from_loc.x
        dy = to_loc.y - from_loc.y
        length = math.sqrt(dx * dx + dy * dy)
        if length < 0.01:
            return
        control = carla.WalkerControl()
        control.speed = self._walk_speed
        control.direction = carla.Vector3D(dx / length, dy / length, 0.0)
        self._walker.apply_control(control)

    def _start_crossing(self, walker_loc, ego_loc):
        """Switch to CROSSING: aim for the closest point on the ego's lane."""
        self._state = self._CROSSING
        cross_target = self._find_road_waypoint(walker_loc, ego_loc)
        if cross_target is None:
            cross_target = ego_loc

        dx = cross_target.x - walker_loc.x
        dy = cross_target.y - walker_loc.y
        length = math.sqrt(dx * dx + dy * dy)
        if length > 0.01:
            self._cross_dir = (dx / length, dy / length)
        else:
            self._cross_dir = (1.0, 0.0)

        self._cross_target = cross_target

    def _find_road_waypoint(self, walker_loc, ego_loc,
                            search_range=80.0, step=2.0):
        """
        Walk along the ego's driving lane (forward and backward) and return
        the carla.Location closest to the pedestrian.
        """
        ego_wp = self._map.get_waypoint(
            ego_loc, project_to_road=True, lane_type=carla.LaneType.Driving
        )
        if ego_wp is None:
            return None

        best_dist = float('inf')
        best_loc = None
        steps = int(search_range / step)

        current = ego_wp
        for _ in range(steps):
            loc = current.transform.location
            d = walker_loc.distance(loc)
            if d < best_dist:
                best_dist = d
                best_loc = loc
            nexts = current.next(step)
            if not nexts:
                break
            current = nexts[0]

        current = ego_wp
        for _ in range(steps):
            loc = current.transform.location
            d = walker_loc.distance(loc)
            if d < best_dist:
                best_dist = d
                best_loc = loc
            prevs = current.previous(step)
            if not prevs:
                break
            current = prevs[0]

        return best_loc

    def terminate(self, new_status):
        """Stop the walker on tree termination."""
        try:
            control = carla.WalkerControl()
            control.speed = 0.0
            self._walker.apply_control(control)
        except Exception:  # pylint: disable=broad-except
            pass
        super().terminate(new_status)


# ---------------------------------------------------------------------------
# Main scenario class
# ---------------------------------------------------------------------------

class ParkingLotPatrol(BasicScenario):
    """
    Ego vehicle spawns in an urban parking lot.
    Three NPC vehicles drive on Traffic Manager autopilot.
    Two pedestrians patrol sidewalk loops and cross the road when the ego
    approaches within PED_TRIGGER_DISTANCE metres.

    Scenario ends on:
      - Any collision involving the ego (terminate_on_failure CollisionTest)
      - Ego reaching within END_PROXIMITY_DISTANCE metres of any entry in
        _END_LOCATIONS (fill this list in to activate the condition)
    """

    timeout = 300  # seconds

    # ---- Tunable parameters ------------------------------------------------
    PED_TRIGGER_DISTANCE = 20.0   # metres: ego proximity that triggers crossing
    PED_WALK_SPEED = 1.4          # m/s
    PED_REACH_DIST = 2.0          # metres: waypoint "reached" threshold
    END_PROXIMITY_DISTANCE = 5.0  # metres: ego-to-end-location that ends scenario
    # ------------------------------------------------------------------------

    # Vehicle spawn transforms
    _VEHICLE_TRANSFORMS = [
        carla.Transform(carla.Location(x=255.28, y=-285.04, z=0.02),
                        carla.Rotation(yaw=90.18)),
        carla.Transform(carla.Location(x=311.56, y=-232.97, z=0.00),
                        carla.Rotation(yaw=90.51)),
        carla.Transform(carla.Location(x=258.32, y=-133.79, z=0.02),
                        carla.Rotation(yaw=-89.82)),
    ]

    # Pedestrian 1 – spawn position + patrol loop
    _PED1_START = carla.Transform(
        carla.Location(x=262.35, y=-182.12, z=0.02),
        carla.Rotation(yaw=-89.82))
    _PED1_PATROL = [
        carla.Transform(carla.Location(x=262.35, y=-182.12, z=0.02),
                        carla.Rotation(yaw=-89.82)),
        carla.Transform(carla.Location(x=262.51, y=-234.02, z=0.02),
                        carla.Rotation(yaw=-89.82)),
        carla.Transform(carla.Location(x=287.93, y=-242.55, z=0.00),
                        carla.Rotation(yaw=359.61)),
        carla.Transform(carla.Location(x=307.62, y=-227.15, z=0.00),
                        carla.Rotation(yaw=90.51)),
        carla.Transform(carla.Location(x=307.28, y=-188.83, z=0.00),
                        carla.Rotation(yaw=90.51)),
        carla.Transform(carla.Location(x=285.99, y=-176.41, z=0.20),
                        carla.Rotation(yaw=180.33)),
    ]

    # Pedestrian 2 – spawn position + patrol loop
    _PED2_START = carla.Transform(
        carla.Location(x=247.66, y=-165.36, z=0.20),
        carla.Rotation(yaw=-179.67))
    _PED2_PATROL = [
        carla.Transform(carla.Location(x=247.66, y=-165.36, z=0.20),
                        carla.Rotation(yaw=-179.67)),
        carla.Transform(carla.Location(x=250.97, y=-145.83, z=0.02),
                        carla.Rotation(yaw=90.18)),
        carla.Transform(carla.Location(x=239.51, y=-126.51, z=0.02),
                        carla.Rotation(yaw=180.92)),
        carla.Transform(carla.Location(x=212.47, y=-138.81, z=0.02),
                        carla.Rotation(yaw=236.97)),
        carla.Transform(carla.Location(x=213.14, y=-165.56, z=0.20),
                        carla.Rotation(yaw=0.33)),
        carla.Transform(carla.Location(x=226.56, y=-165.48, z=0.20),
                        carla.Rotation(yaw=0.33)),
    ]

    # End locations – add carla.Location(...) entries here.
    # The scenario ends when the ego comes within END_PROXIMITY_DISTANCE metres
    # of any of these locations.  Leave empty to disable this condition.
    _END_LOCATIONS = [
        # Example: carla.Location(x=270.0, y=-200.0, z=0.0),
    ]

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=300):
        self.timeout = timeout
        self._world = world
        self._map = CarlaDataProvider.get_map()

        self._vehicles = []
        self._pedestrians = []

        super().__init__("ParkingLotPatrol", ego_vehicles, config, world,
                         debug_mode, criteria_enable=criteria_enable)

    def _setup_scenario_trigger(self, config):
        """
        Skip the default InTimeToArrivalToLocation trigger.
        The ego spawns directly at the start point so the scenario begins
        immediately without waiting for the ego to reach a trigger location.
        """
        return None

    def _initialize_actors(self, config):
        """
        Spawn NPC vehicles (autopilot via Traffic Manager) and pedestrians.
        Pedestrians are controlled via WalkerControl in PedestrianBehavior;
        no AI walker controller is spawned.
        """
        tm_port = CarlaDataProvider.get_traffic_manager_port()

        # --- NPC vehicles on autopilot ---
        for transform in self._VEHICLE_TRANSFORMS:
            vehicle = CarlaDataProvider.request_new_actor('vehicle.*', transform)
            if vehicle is None:
                raise RuntimeError(
                    "Could not spawn vehicle at {}".format(transform.location))
            vehicle.set_autopilot(True, tm_port)
            self._vehicles.append(vehicle)
            self.other_actors.append(vehicle)

        # --- Pedestrians (no AI controller – WalkerControl used in behavior) ---
        for ped_transform in (self._PED1_START, self._PED2_START):
            walker = CarlaDataProvider.request_new_actor(
                'walker.pedestrian.*', ped_transform, rolename='scenario')
            if walker is None:
                raise RuntimeError(
                    "Could not spawn pedestrian at {}".format(ped_transform.location))
            self._pedestrians.append(walker)
            self.other_actors.append(walker)

    def _create_behavior(self):
        """
        Behavior tree:

            Parallel (SUCCESS_ON_ONE)
            ├── PedestrianBehavior_1   (RUNNING forever)
            ├── PedestrianBehavior_2   (RUNNING forever)
            └── EndLocationTriggers    (SUCCESS when ego nears any end location)
                └── InTriggerDistanceToLocation × N

        Collision termination is handled via CollisionTest in _create_test_criteria.
        """
        root = py_trees.composites.Parallel(
            "ParkingLotPatrol_Main",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )

        root.add_child(PedestrianBehavior(
            self._pedestrians[0],
            self.ego_vehicles[0],
            self._PED1_PATROL,
            trigger_dist=self.PED_TRIGGER_DISTANCE,
            walk_speed=self.PED_WALK_SPEED,
            reach_dist=self.PED_REACH_DIST,
            name="PedestrianBehavior_1"
        ))

        # root.add_child(PedestrianBehavior(
        #     self._pedestrians[1],
        #     self.ego_vehicles[0],
        #     self._PED2_PATROL,
        #     trigger_dist=self.PED_TRIGGER_DISTANCE,
        #     walk_speed=self.PED_WALK_SPEED,
        #     reach_dist=self.PED_REACH_DIST,
        #     name="PedestrianBehavior_2"
        # ))

        if self._END_LOCATIONS:
            end_triggers = py_trees.composites.Parallel(
                "EndLocationTriggers",
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
            )
            for i, loc in enumerate(self._END_LOCATIONS):
                end_triggers.add_child(
                    InTriggerDistanceToLocation(
                        self.ego_vehicles[0],
                        loc,
                        self.END_PROXIMITY_DISTANCE,
                        name="EndTrigger_{}".format(i)
                    )
                )
            root.add_child(end_triggers)

        return root

    def _create_test_criteria(self):
        """Fail and terminate immediately on any collision involving the ego."""
        return [CollisionTest(self.ego_vehicles[0], terminate_on_failure=True)]

    def __del__(self):
        self.remove_all_actors()
