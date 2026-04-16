# Scenario Runner Project Memory

## Current Goal

Keep `AccidentAhead`, `AccidentUpcoming`, and `Introduction` stable for CARLA 0.9.15 runs, especially with nDisplay launches.

## Launch Commands

- AccidentAhead normal test:
  `python scenario_runner.py --scenario AccidentAhead_High_1 --reloadWorld --frameRate 60 --sync --participant-id 3`
- AccidentUpcoming normal test:
  `python scenario_runner.py --scenario AccidentUpcoming_High_1 --reloadWorld --frameRate 60 --sync --participant-id 3`

## Recovery Notes

- `srunner/scenarios/accident_ahead.py` was damaged by an accidental checkout.
- The good saved snapshot was `srunner/scenarios/claude_sucks.py`.
- `accident_ahead.py` was restored from that snapshot on 2026-04-16.
- Backup files created during recovery:
  - `srunner/scenarios/accident_ahead.destroyed_20260416_123507.py`
  - `srunner/scenarios/accident_ahead.pre_claude_sucks_20260416_124025.py`
- These backup files are for recovery only and should not be used as scenario modules.

## Visual Navigation

- We want visual navigation cues, not active audio cue guidance.
- Visual markers should be screaming pink.
- In `AccidentAhead`, `_DEBUG_DRAW_NAV_MARKERS = True`.
- In `AccidentAhead`, `_DEBUG_DRAW_SOUNDCUES = False`.
- In `AccidentUpcoming`, the audio guidance branch is commented out and visual marker branch is active.
- `NavigationMarker` color lives in `srunner/scenariomanager/scenarioatomics/atomic_behaviors_custom.py`.

## Traffic Lights

- `KeepTrafficLightsGreen` should not freeze lights permanently green.
- Current intended behavior is short traffic light cycles:
  - green: `6.0 s`
  - yellow: `1.0 s`
  - red: `2.0 s`
- This preserves some red/yellow while avoiding long waits.

## AccidentAhead Important State

- Restored from `claude_sucks.py`.
- Uses `KeepTrafficLightsGreen`.
- Uses `EgoVehicleLSLStream`.
- Uses visual navigation markers.
- Audio cue branch is commented out.
- Construction site pedestrian event should not be able to fail the whole scenario.
- Construction site pedestrian tuning:
  - trigger location currently `x=203.48, y=-237.06, z=0.02`
  - trigger distance currently `9.0 m`
  - run speed currently `5.5 m/s`
- Flow spacing was widened to reduce traffic density.
- Ego speed cap is `40.0 km/h`.

## AccidentUpcoming Important State

- Audio cue branch is commented out.
- Visual navigation markers are active.
- `KeepTrafficLightsGreen` is attached.
- `TrafficManager` seed is explicitly set to `TRAFFIC_BATCH_SEED`.
- First bike near-miss was tuned:
  - shared sync point around `x=127.64, y=-179.67`
  - `time_lead=0.2`
  - trigger radius reduced from `10.0 m` to `8.0 m`
- Final car event was removed.
- Final pedestrian event was retuned near the parked bus.
- Batch 3 static bus uses `vehicle.mitsubishi.fusorosa` with right blinker.

## Custom Behavior Notes

- `StartWalkerControllers` supports `destination_mode="far_nav"`.
- `WalkerWalkTo` returns `SUCCESS` if the walker is missing/dead so optional events do not terminate the whole scenario.
- `BikeNearMissEvent` is kinematic/sliding by design for stability.

## Git Hygiene

- Do not commit recovery scratch files unless explicitly needed.
- Avoid committing generated folders like `tts_output/`.
- Avoid committing runtime logs like `sr_out.txt`.
- Check `git status --short` before every commit.
