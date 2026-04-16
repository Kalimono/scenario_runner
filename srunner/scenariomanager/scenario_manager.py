#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the ScenarioManager implementation.
It must not be modified and is for reference only!
"""

from __future__ import print_function
import sys
import time
import traceback

import py_trees

from srunner.autoagents.agent_wrapper import AgentWrapper
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.result_writer import ResultOutputProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog


class ScenarioManager(object):

    """
    Basic scenario manager class. This class holds all functionality
    required to start, and analyze a scenario.

    The user must not modify this class.

    To use the ScenarioManager:
    1. Create an object via manager = ScenarioManager()
    2. Load a scenario via manager.load_scenario()
    3. Trigger the execution of the scenario manager.run_scenario()
       This function is designed to explicitly control start and end of
       the scenario execution
    4. Trigger a result evaluation with manager.analyze_scenario()
    5. If needed, cleanup with manager.stop_scenario()
    """

    def __init__(self, debug_mode=False, sync_mode=False, timeout=2.0):
        """
        Setups up the parameters, which will be filled at load_scenario()

        """
        self.scenario = None
        self.scenario_tree = None
        self.ego_vehicles = None
        self.other_actors = None

        self._debug_mode = debug_mode
        self._agent = None
        self._sync_mode = sync_mode
        self._watchdog = None
        self._timeout = timeout

        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None

    def _reset(self):
        """
        Reset all parameters
        """
        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        GameTime.restart()

    def cleanup(self):
        """
        This function triggers a proper termination of a scenario
        """

        if self._watchdog is not None:
            self._watchdog.stop()
            self._watchdog = None

        if self.scenario is not None:
            self.scenario.terminate()

        if self._agent is not None:
            self._agent.cleanup()
            self._agent = None

        CarlaDataProvider.cleanup()

    def load_scenario(self, scenario, agent=None):
        """
        Load a new scenario
        """
        self._reset()
        self._agent = AgentWrapper(agent) if agent else None
        if self._agent is not None:
            self._sync_mode = True
        self.scenario = scenario
        self.scenario_tree = self.scenario.scenario_tree
        self.ego_vehicles = scenario.ego_vehicles
        self.other_actors = scenario.other_actors

        # To print the scenario tree uncomment the next line
        # py_trees.display.render_dot_tree(self.scenario_tree)

        if self._agent is not None:
            self._agent.setup_sensors(self.ego_vehicles[0], self._debug_mode)

    def run_scenario(self):
        """
        Trigger the start of the scenario and wait for it to finish/fail
        """
        print("ScenarioManager: Running scenario {}".format(self.scenario_tree.name))
        self.start_system_time = time.time()
        start_game_time = GameTime.get_time()

        self._watchdog = Watchdog(float(self._timeout))
        self._watchdog.start()
        self._running = True
        target_dt = None
        next_tick_deadline = None
        if self._sync_mode:
            try:
                ws = CarlaDataProvider.get_world().get_settings()
                if ws.fixed_delta_seconds and ws.fixed_delta_seconds > 0:
                    target_dt = float(ws.fixed_delta_seconds)
                    next_tick_deadline = time.perf_counter() + target_dt
            except Exception:  # pylint: disable=broad-except
                pass

        try:
          while self._running:
            timestamp = None
            world = CarlaDataProvider.get_world()
            try:
                if world:
                    snapshot = world.get_snapshot()
                    if snapshot:
                        timestamp = snapshot.timestamp
            except Exception:  # pylint: disable=broad-except
                print("[ScenarioManager] EXCEPTION in get_snapshot():")
                traceback.print_exc()
                self._running = False
                break
            if timestamp:
                try:
                    self._tick_scenario(timestamp)
                except KeyboardInterrupt:
                    print("[ScenarioManager] KeyboardInterrupt caught — watchdog timeout or Ctrl+C")
                    print("[ScenarioManager] Watchdog status: {}".format(
                        self._watchdog.get_status() if self._watchdog else "N/A"))
                    traceback.print_exc()
                    self._running = False
                    break
                except Exception:  # pylint: disable=broad-except
                    print("[ScenarioManager] EXCEPTION in _tick_scenario (outer catch):")
                    traceback.print_exc()
                    self._running = False
                    break

            if target_dt is not None and self._running:
                self._pace_sync_loop(next_tick_deadline, target_dt)
                next_tick_deadline += target_dt
                now = time.perf_counter()
                if now > next_tick_deadline + target_dt:
                    next_tick_deadline = now + target_dt

        except KeyboardInterrupt:
            print("[ScenarioManager] KeyboardInterrupt at top-level while loop — watchdog fired or Ctrl+C")
            print("[ScenarioManager] Watchdog status: {}".format(
                self._watchdog.get_status() if self._watchdog else "N/A"))
            traceback.print_exc()
        except SystemExit as e:
            print("[ScenarioManager] SystemExit({}) caught at top-level".format(e.code))
            traceback.print_exc()
        except BaseException as e:
            print("[ScenarioManager] BaseException caught at top-level: {} {}".format(type(e).__name__, e))
            traceback.print_exc()

        print("[ScenarioManager] Main loop exited. Tree status: {} Watchdog OK: {}".format(
            self.scenario_tree.status,
            self._watchdog.get_status() if self._watchdog else "N/A"))
        self.cleanup()

        self.end_system_time = time.time()
        end_game_time = GameTime.get_time()

        self.scenario_duration_system = self.end_system_time - \
            self.start_system_time
        self.scenario_duration_game = end_game_time - start_game_time

        if self.scenario_tree.status == py_trees.common.Status.FAILURE:
            print("ScenarioManager: Terminated due to failure")

    @staticmethod
    def _pace_sync_loop(deadline, target_dt):
        """
        Sleep coarsely, then yield in short slices until the next frame
        deadline. If the loop is already behind, skip waiting entirely.
        """
        now = time.perf_counter()
        if now >= deadline:
            return

        coarse_sleep_margin = min(0.002, target_dt * 0.25)
        while True:
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                return
            if remaining > coarse_sleep_margin:
                time.sleep(remaining - coarse_sleep_margin)
            else:
                time.sleep(0)

    def _tick_scenario(self, timestamp):
        """
        Run next tick of scenario and the agent.
        If running synchornously, it also handles the ticking of the world.
        """

        if self._timestamp_last_run < timestamp.elapsed_seconds and self._running:
            self._timestamp_last_run = timestamp.elapsed_seconds

            self._watchdog.update()

            if self._debug_mode:
                print("\n--------- Tick ---------\n")

            # Update game time and actor information
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()

            if self._agent is not None:
                ego_action = self._agent()  # pylint: disable=not-callable

            if self._agent is not None:
                self.ego_vehicles[0].apply_control(ego_action)

            # Tick scenario
            try:
                self.scenario_tree.tick_once()
            except Exception as tick_exc:  # pylint: disable=broad-except
                print("\n[ScenarioManager] EXCEPTION during scenario_tree.tick_once():")
                traceback.print_exc()
                print("[ScenarioManager] Tree state at crash:")
                try:
                    py_trees.display.print_ascii_tree(self.scenario_tree, show_status=True)
                except Exception:  # pylint: disable=broad-except
                    print("[ScenarioManager] (could not print tree)")
                print("[ScenarioManager] Re-raising — scenario will stop.")
                raise

            if self._debug_mode:
                print("\n")
                py_trees.display.print_ascii_tree(self.scenario_tree, show_status=True)
                sys.stdout.flush()

            if self.scenario_tree.status != py_trees.common.Status.RUNNING:
                print("[ScenarioManager] Tree stopped with status: {}".format(
                    self.scenario_tree.status))
                print("[ScenarioManager] Children statuses:")
                for child in self.scenario_tree.children:
                    print("[ScenarioManager]   '{}' -> {}".format(child.name, child.status))
                    if hasattr(child, 'children'):
                        for subchild in child.children:
                            print("[ScenarioManager]     '{}' -> {}".format(subchild.name, subchild.status))
                self._running = False

        if self._sync_mode and self._running and self._watchdog.get_status():
            try:
                CarlaDataProvider.get_world().tick()
            except Exception:  # pylint: disable=broad-except
                print("[ScenarioManager] EXCEPTION in world.tick():")
                traceback.print_exc()
                self._running = False

    def get_running_status(self):
        """
        returns:
           bool:  False if watchdog exception occured, True otherwise
        """
        return self._watchdog.get_status()

    def stop_scenario(self):
        """
        This function is used by the overall signal handler to terminate the scenario execution
        """
        self._running = False

    def analyze_scenario(self, stdout, filename, junit, json):
        """
        This function is intended to be called from outside and provide
        the final statistics about the scenario (human-readable, in form of a junit
        report, etc.)
        """

        failure = False
        timeout = False
        result = "SUCCESS"

        criteria = self.scenario.get_criteria()
        if len(criteria) == 0:
            print("Nothing to analyze, this scenario has no criteria")
            return True

        for criterion in criteria:
            if (not criterion.optional and
                    criterion.test_status != "SUCCESS" and
                    criterion.test_status != "ACCEPTABLE"):
                failure = True
                result = "FAILURE"
            elif criterion.test_status == "ACCEPTABLE":
                result = "ACCEPTABLE"

        if self.scenario.timeout_node.timeout and not failure:
            timeout = True
            result = "TIMEOUT"

        output = ResultOutputProvider(self, result, stdout, filename, junit, json)
        output.write()

        return failure or timeout
