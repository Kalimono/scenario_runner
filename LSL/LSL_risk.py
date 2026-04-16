import argparse
import json
import os
import socket
import sys
import time
import importlib.util
import random
import traceback
from pylsl import StreamInfo, StreamOutlet
from dataclasses import asdict

class LSLOutlet:
    def __init__(self, config_path, stream_type, channel_format, nominal_srate=0.0):
        with open(config_path, "r") as f:
            config = json.load(f)

        self.labels = self._parse_channel_labels(config.get("ChannelModalities", {}))
        self.outlet = StreamOutlet(
            self._create_stream_info(config, stream_type, channel_format, nominal_srate)
        )

    def _parse_channel_labels(self, modalities):
        sorted_keys = sorted(
            modalities, key=lambda k: int("".join(filter(str.isdigit, k)))
        )
        return [modalities[k] for k in sorted_keys]

    def _create_stream_info(self, config, stream_type, channel_format, nominal_srate):
        info = StreamInfo(
            name=config.get("NotebookDeviceName", "DefaultStream"),
            type=stream_type,
            channel_count=len(self.labels),
            nominal_srate=nominal_srate,
            channel_format=channel_format,
            source_id=config.get("SourceID", "DefaultSourceID"),
        )

        channels = info.desc().append_child("channels")
        for label in self.labels:
            channels.append_child("channel").append_child_value("label", label)

        return info

    def push_sample(self, data):
        try:
            sample = [data[label] for label in self.labels]
        except KeyError as e:
            raise ValueError(f"Missing required channel in data: {e.args[0]}")
        self.outlet.push_sample(sample)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-port",
        dest="port",
        type=int,
        default=54017,
        help="UDP port to receive data (default: 54017)", # This text had another number until 260304
    )
    parser.add_argument(
        "-dt",
        dest="data_type",
        type=str,
        default="lsl_risk_assessment",
        help="Type of data to send over LSL (default: lsl_risk_assessment)",
    )
    return parser.parse_args()


def setup_socket(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", port))
    return sock

# Receives hmi states as json messages, convert values to numbers, and forwards them on LSL
# No sleep, throttling can instead be set by push_interval
def main():
    args = parse_args()
    sock = setup_socket(args.port)
    lsl = LSLOutlet(
        f"lsl_connection/{args.data_type}.json",
        stream_type=args.data_type,
        channel_format="float32",
    )
    buffer_size = 8192
    last_push_time = 0
    push_interval = 0.1 # Set to 0 for pushing data for every received message
    while True:
        try:
            buffer, _ = sock.recvfrom(buffer_size)
            if time.time() - last_push_time >= push_interval:
                state_dict = json.loads(buffer.decode("utf-8"))
                numdict = {
                    "risk_level":state_dict.get("risk_level", 0),
                    "driving_phase":state_dict.get("driving_phase", 0),
                    "driver_distracted": 1 if state_dict.get("driver_distracted") else 0,
                }
                lsl.push_sample(numdict)
                last_push_time = time.time()
        except KeyboardInterrupt:
            exit()
        except:
            traceback.print_exc()

if __name__ == "__main__":
    main()
