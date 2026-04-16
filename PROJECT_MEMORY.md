# Scenario Runner Project Memory

This file is the handoff note for future Codex sessions. Read it before changing
the IAIMS scenarios.

## Project Context

- Repository root: `C:\scenario_runner`
- Main branch in use: `iaims-scenarios`
- CARLA version: `0.9.15`
- Primary scenarios:
  - `srunner/scenarios/accident_ahead.py`
  - `srunner/scenarios/accident_upcoming.py`
  - `srunner/scenarios/introduction.py`
- Primary scenario XML files:
  - `srunner/examples/AccidentAhead.xml`
  - `srunner/examples/AccidentUpcoming.xml`
  - `srunner/examples/Introduction.xml`
- nDisplay runs are important. Do not assume behavior that only works in the normal spectator view is sufficient.

## Launch Commands

- AccidentAhead high-load normal run:
  `python scenario_runner.py --scenario AccidentAhead_High_1 --reloadWorld --frameRate 60 --sync --participant-id 3`
- AccidentUpcoming high-load normal run:
  `python scenario_runner.py --scenario AccidentUpcoming_High_1 --reloadWorld --frameRate 60 --sync --participant-id 3`
- Introduction:
  `python scenario_runner.py --scenario Introduction_1 --reloadWorld --frameRate 60 --sync --participant-id 3`

## Important Recovery History

- On 2026-04-16, `srunner/scenarios/accident_ahead.py` was damaged by an accidental `git checkout --`.
- A saved good snapshot existed at `srunner/scenarios/claude_sucks.py`.
- `accident_ahead.py` was restored from `claude_sucks.py`.
- Recovery backups created:
  - `srunner/scenarios/accident_ahead.destroyed_20260416_123507.py`
  - `srunner/scenarios/accident_ahead.pre_claude_sucks_20260416_124025.py`
- These backup files are scratch/recovery artifacts. Do not import them as scenario modules.
- Recovery commit pushed:
  `219718d Recover IAIMS scenarios and add project memory`

## Current Navigation Philosophy

- Use visual navigation markers, not active audio guidance.
- Audio cue branches should remain commented out unless explicitly re-enabled.
- Do not show unused audio cue debug markers.
- Visual markers should be screaming pink.
- Shared marker color lives in:
  `srunner/scenariomanager/scenarioatomics/atomic_behaviors_custom.py`
  class: `NavigationMarker`
- AccidentAhead currently has `_DEBUG_DRAW_NAV_MARKERS = True` so all visual marker positions are visible.
- AccidentAhead currently has `_DEBUG_DRAW_SOUNDCUES = False`.
- AccidentUpcoming has the visual marker branch attached. Its audio branch is commented out.

## Traffic Lights

- Do not freeze every light permanently green.
- CARLA traffic lights are grouped and can behave strangely if frozen at the wrong state.
- Current intended behavior is short cycles via `KeepTrafficLightsGreen`:
  - green: `6.0 s`
  - yellow: `1.0 s`
  - red: `2.0 s`
- This preserves visible red/yellow while preventing long waits.
- `KeepTrafficLightsGreen` lives in:
  `srunner/scenariomanager/scenarioatomics/atomic_behaviors_custom.py`

## AccidentAhead State

- Restored from `claude_sucks.py`.
- Uses `EgoVehicleLSLStream`.
- Uses `KeepTrafficLightsGreen`.
- Uses visual navigation markers.
- Audio guidance branch is commented out.
- Ego speed cap: `40.0 km/h`.
- `TrafficManager` seeding should stay deterministic.
- Flow spacing was widened to reduce traffic density.
- Construction-site pedestrian event:
  - function: `_create_cs2_ped_behavior`
  - trigger location: `x=203.48, y=-237.06, z=0.02`
  - trigger distance: `9.0 m`
  - run speed: `5.5 m/s`
  - this event must never be able to terminate the whole scenario if the walker is missing/dead
- `WalkerWalkTo` returns `SUCCESS` when the walker is missing/dead to prevent optional pedestrian events from killing the scenario.
- Important recent failure:
  - `CS2_PedRun -> Status.FAILURE` caused full scenario termination.
  - Fixed by making `WalkerWalkTo` non-fatal for missing/dead walkers.
- Visual navigation markers should be pink only, not gold/yellow.

## AccidentAhead Bike Event

- The bike near-miss is intentionally kinematic/sliding for stability.
- Do not switch it back to physics-driven bike movement unless explicitly requested.
- Current debug-only mode should remain off:
  `_DEBUG_LAST_EVENT = False`
- The bike event has previously been tuned to:
  - fixed speed around `7.0 m/s`
  - trigger within `5 m` of activation point
  - no ego-speed-dependent behavior

## AccidentUpcoming State

- Audio cue branch is commented out.
- Visual navigation markers are active.
- `KeepTrafficLightsGreen` is attached.
- `TrafficManager` seed is explicitly set to `TRAFFIC_BATCH_SEED`.
- `_DEBUG_FIRST_BIKE_ONLY = False` for normal runs.
- First bike near-miss tuning:
  - shared sync point around `x=127.64, y=-179.67`
  - `time_lead=0.2`
  - trigger radius reduced from `10.0 m` to `8.0 m`
  - do not remove max speed entirely; that broke behavior before
- Final event:
  - final car event was removed
  - final pedestrian event was retuned near the parked bus
  - batch 3 static bus uses `vehicle.mitsubishi.fusorosa`
  - bus has right blinker enabled

## Introduction State

- Uses visual navigation markers.
- Marker color should match the screaming pink style.
- This scenario also imports `KeepTrafficLightsGreen`, `NavigationMarker`, and `SetEgoMaxSpeed`.

## Custom Behavior Notes

- `StartWalkerControllers` supports `destination_mode="far_nav"`.
- `WalkerWalkTo` should not return `FAILURE` just because an optional walker is unavailable.
- `BikeNearMissEvent` is kinematic by design.
- `SpawnActorGroup` is used for triggered mixed batches.
- `NavigationMarker` is the shared sequential visual marker behavior.
- `SetEgoMaxSpeed` is used by scenario speed zones.
- `KeepTrafficLightsGreen` is intentionally misnamed now; it actually shortens the cycle rather than forcing green.

## LSL Notes

- LSL support files are under `LSL/`.
- `EgoVehicleLSLStream` runs in a background thread and should stop gracefully if the ego actor dies.
- Scenario logs may show:
  `[EgoVehicleLSLStream] Ego actor no longer alive - stopping stream.`
- This is expected on scenario shutdown.

## CARLA/NDisplay Gotchas

- `world.get_spectator()` can fail in some launch modes. Spectator movement is optional and should never crash scenario loading.
- `client.reload_world()` may fail with `Map '' not found`; `scenario_runner.py` has fallback cleanup logic for this.
- CARLA can get into bad sync/latency states. Restarting the simulator or machine has helped.
- Avoid heavy persistent debug drawing unless intentionally testing markers.
- Missing/dead actors should usually cause optional event branches to no-op, not fail the root tree.

## Git Hygiene

- Always run `git status --short` before committing.
- Do not use `git checkout --` or destructive reset commands unless the user explicitly asks and the target is fully understood.
- Do not commit scratch/recovery files unless explicitly needed:
  - `srunner/scenarios/claude_sucks.py`
  - `srunner/scenarios/accident_ahead.destroyed_*.py`
  - `srunner/scenarios/accident_ahead.pre_claude_sucks_*.py`
  - `accident_ahead_export/`
  - `sr_out.txt`
  - `tts_output/`
- Avoid committing generated caches like `__pycache__/`.
- Old deleted MP3 files may appear in `git status`; do not assume they should be committed.
- Prefer targeted commits with explicit file lists.

## Verification Checklist

Before saying a scenario is healthy:

- Run:
  `python -m py_compile srunner/scenarios/accident_ahead.py`
- Run:
  `python -m py_compile srunner/scenarios/accident_upcoming.py`
- Run:
  `python -m py_compile srunner/scenarios/introduction.py`
- Run:
  `python -m py_compile srunner/scenariomanager/scenarioatomics/atomic_behaviors_custom.py`
- Import check when useful:
  `python -c "import srunner.scenarios.accident_ahead; print('accident_ahead import ok')"`
- If CARLA is running, do a short launch test with the commands above.

## Human Preference Notes

- The user wants practical fixes, not theory.
- Keep CARLA-related changes robust and non-fatal.
- Preserve scenario stability over perfect animation.
- If a debug/test setup is added, turn it off before calling the scenario ready.
- If future recovery is needed, check this file first.
