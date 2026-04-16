"""
Background-threaded LSL stream for CARLA ego-vehicle telemetry.

Usage from a scenario (or anywhere with access to the ego actor):

    from LSL.ego_vehicle_stream import EgoVehicleLSLStream

    stream = EgoVehicleLSLStream(ego_actor, participant_id="42")
    stream.start()
    ...
    stream.stop()

The stream runs in a daemon thread and does not touch the scenario
behavior tree at all.
"""

import math
import os
import sys
import threading
import time
import traceback

_LSL_DIR = os.path.dirname(os.path.abspath(__file__))
if _LSL_DIR not in sys.path:
    sys.path.insert(0, _LSL_DIR)

try:
    from LSL_risk import LSLOutlet
    _LSL_AVAILABLE = True
except Exception as _exc:
    _LSL_AVAILABLE = False
    print("[EgoVehicleLSLStream] pylsl/LSLOutlet not available: {}".format(_exc), flush=True)

_DEFAULT_CONFIG = os.path.join(_LSL_DIR, "lsl_ego_vehicle.json")


class EgoVehicleLSLStream:
    """
    Streams ego-vehicle telemetry to LSL in a background daemon thread.

    24 float32 channels:
      speed_kmh, velocity_{x,y,z}, acceleration_{x,y,z},
      throttle, steer, brake, hand_brake, reverse, gear,
      location_{x,y,z}, rotation_{pitch,yaw,roll},
      angular_velocity_{x,y,z}, speed_limit, participant_id
    """

    def __init__(self, ego_actor, participant_id="", rate_hz=30.0,
                 config_path=None):
        self._ego = ego_actor
        self._participant_id = self._parse_participant_id(participant_id)
        self._interval = 1.0 / rate_hz
        self._config_path = config_path or _DEFAULT_CONFIG
        self._thread = None
        self._stop_event = threading.Event()

    # ------------------------------------------------------------------

    def start(self):
        if not _LSL_AVAILABLE:
            print("[EgoVehicleLSLStream] LSL not available — "
                  "telemetry will not be streamed.", flush=True)
            return
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    # ------------------------------------------------------------------

    def _run(self):
        try:
            outlet = LSLOutlet(
                self._config_path,
                stream_type="ego_vehicle",
                channel_format="float32",
            )
            print("[EgoVehicleLSLStream] Started — {} channels @ ~{:.0f} Hz, "
                  "participant_id={}".format(
                      len(outlet.labels), 1.0 / self._interval,
                      self._participant_id), flush=True)
        except Exception as exc:
            print("[EgoVehicleLSLStream] Failed to create LSL outlet: "
                  "{}".format(exc), flush=True)
            traceback.print_exc()
            return

        while not self._stop_event.is_set():
            # Guard: stop if the actor has been destroyed.
            if not getattr(self._ego, 'is_alive', False):
                print("[EgoVehicleLSLStream] Ego actor no longer alive — "
                      "stopping stream.", flush=True)
                break

            try:
                vel = self._ego.get_velocity()
                acc = self._ego.get_acceleration()
                ctrl = self._ego.get_control()
                tf = self._ego.get_transform()
                ang = self._ego.get_angular_velocity()

                speed_kmh = 3.6 * math.sqrt(
                    vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

                try:
                    speed_limit = self._ego.get_speed_limit()
                except Exception:
                    speed_limit = 0.0

                sample = {
                    "speed_kmh":          speed_kmh,
                    "velocity_x":         vel.x,
                    "velocity_y":         vel.y,
                    "velocity_z":         vel.z,
                    "acceleration_x":     acc.x,
                    "acceleration_y":     acc.y,
                    "acceleration_z":     acc.z,
                    "throttle":           ctrl.throttle,
                    "steer":              ctrl.steer,
                    "brake":              ctrl.brake,
                    "hand_brake":         1.0 if ctrl.hand_brake else 0.0,
                    "reverse":            1.0 if ctrl.reverse else 0.0,
                    "gear":               float(ctrl.gear),
                    "location_x":         tf.location.x,
                    "location_y":         tf.location.y,
                    "location_z":         tf.location.z,
                    "rotation_pitch":     tf.rotation.pitch,
                    "rotation_yaw":       tf.rotation.yaw,
                    "rotation_roll":      tf.rotation.roll,
                    "angular_velocity_x": ang.x,
                    "angular_velocity_y": ang.y,
                    "angular_velocity_z": ang.z,
                    "speed_limit":        speed_limit,
                    "participant_id":     self._participant_id,
                }
                outlet.push_sample(sample)
            except RuntimeError:
                # CARLA raises RuntimeError when the actor is gone.
                print("[EgoVehicleLSLStream] Ego actor destroyed — "
                      "stopping stream.", flush=True)
                break
            except Exception:
                # Transient error (network hiccup, etc.) — skip this sample.
                pass

            self._stop_event.wait(self._interval)

        print("[EgoVehicleLSLStream] Stopped.", flush=True)

    # ------------------------------------------------------------------

    @staticmethod
    def _parse_participant_id(pid):
        if not pid:
            return 0.0
        try:
            return float(pid)
        except (ValueError, TypeError):
            return float(abs(hash(pid)) % (10 ** 8))
