#!/usr/bin/env python

import argparse
import glob
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import carla


DEFAULT_LAUNCHER = r"C:\Users\Smarteye\Documents\launch_node_2_single.bat"
DEFAULT_SCENARIO_RUNNER = r"C:\scenario_runner\scenario_runner.py"
DEFAULT_EXAMPLES_GLOB = r"C:\scenario_runner\srunner\examples\*.xml"


def find_scenario_town(scenario_name):
    for path in glob.glob(DEFAULT_EXAMPLES_GLOB):
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError:
            continue

        for scenario in root.findall("scenario"):
            if scenario.attrib.get("name") == scenario_name:
                return scenario.attrib.get("town"), path

    return None, None


def wait_for_world(host, port, expected_town, timeout_seconds):
    deadline = time.time() + timeout_seconds
    last_error = None

    while time.time() < deadline:
        try:
            client = carla.Client(host, port)
            client.set_timeout(5.0)
            world = client.get_world()
            current_map = world.get_map().name.split("/")[-1]
            if current_map == expected_town:
                return True, world.get_map().name
            last_error = "connected to wrong map '{}'".format(world.get_map().name)
        except RuntimeError as exc:
            last_error = str(exc)

        time.sleep(1.0)

    return False, last_error


def main():
    parser = argparse.ArgumentParser(
        description="Launch nDisplay CARLA on the scenario map, wait for readiness, then run ScenarioRunner."
    )
    parser.add_argument("--scenario", required=True, help="ScenarioRunner scenario name, for example AccidentAhead_1")
    parser.add_argument("--host", default="127.0.0.1", help="CARLA host")
    parser.add_argument("--port", default=2000, type=int, help="CARLA RPC port")
    parser.add_argument("--launcher", default=DEFAULT_LAUNCHER, help="Path to the nDisplay launcher batch file")
    parser.add_argument(
        "--scenario-runner",
        default=DEFAULT_SCENARIO_RUNNER,
        help="Path to scenario_runner.py",
    )
    parser.add_argument(
        "--startup-timeout",
        default=90.0,
        type=float,
        help="Seconds to wait for CARLA to come up on the expected map",
    )
    parser.add_argument(
        "scenario_runner_args",
        nargs=argparse.REMAINDER,
        help="Additional arguments passed through to scenario_runner.py. Prefix them with --",
    )
    args = parser.parse_args()

    town, config_path = find_scenario_town(args.scenario)
    if not town:
        print("ERROR: Could not find scenario '{}' in {}".format(args.scenario, DEFAULT_EXAMPLES_GLOB))
        return 1

    if not os.path.exists(args.launcher):
        print("ERROR: Launcher not found: {}".format(args.launcher))
        return 1

    if not os.path.exists(args.scenario_runner):
        print("ERROR: ScenarioRunner not found: {}".format(args.scenario_runner))
        return 1

    print("Scenario: {}".format(args.scenario))
    print("Scenario config: {}".format(config_path))
    print("Required town: {}".format(town))
    print("Launcher: {}".format(args.launcher))

    subprocess.Popen(
        ["cmd", "/c", args.launcher, town],
        cwd=os.path.dirname(args.launcher) or None,
    )

    ok, detail = wait_for_world(args.host, args.port, town, args.startup_timeout)
    if not ok:
        print("ERROR: CARLA did not become ready on '{}' within {:.1f}s ({})".format(
            town, args.startup_timeout, detail))
        return 1

    print("CARLA ready on {}".format(detail))

    forwarded_args = list(args.scenario_runner_args)
    if forwarded_args and forwarded_args[0] == "--":
        forwarded_args = forwarded_args[1:]

    command = [
        sys.executable,
        args.scenario_runner,
        "--scenario",
        args.scenario,
        "--sync",
        "--reloadWorld",
    ] + forwarded_args

    print("Running: {}".format(" ".join(command)))
    completed = subprocess.run(command, cwd=os.path.dirname(args.scenario_runner) or None)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
