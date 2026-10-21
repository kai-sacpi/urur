#!/usr/bin/env python3
"""Run MagLoc known-background tracking and publish its position to ROS 2."""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np


def _bootstrap_magloc_root(raw_root: str) -> Path:
    root = Path(raw_root).expanduser().resolve()
    if not (root / "magloc").is_dir():
        raise ValueError(f"MagLoc package directory does not exist: {root / 'magloc'}")
    sys.path.insert(0, str(root))
    return root


def _parse_bootstrap_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--magloc-root",
        default=os.environ.get(
            "MAGLOC_ROOT", "/home/huaxin/Codes/magslam-realtime-explore"
        ),
    )
    return parser.parse_known_args(argv)


def main(argv: list[str] | None = None) -> int:
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    bootstrap, remaining = _parse_bootstrap_args(raw_argv)
    magloc_root = _bootstrap_magloc_root(bootstrap.magloc_root)

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from magloc.cli.track_known_background_dipole import (
        _build_known_background_overrides,
        build_arg_parser,
    )
    from magloc.configuration import config_use
    from magloc.domain import MeasurementFrame, MeasurementPacket, load_sensor_array_csv
    from magloc.domain.state_layout import POS_IDX
    from magloc.numeric import (
        NUMERIC_THREAD_ENV_VAR,
        configure_numeric_threads,
    )
    from magloc.pipeline.runtime import run_pipeline
    from magloc.sinks.base import BaseSink
    from magloc.sinks.builders import SinkRegistry
    from magloc.sources.base import BaseSource
    from magloc.sources.builders import SourceRegistry
    from rclpy.qos import qos_profile_sensor_data
    from hx_waypoint_nucleo import (
        HubStreamParser,
        NucleoCaptureConfig,
        NucleoCaptureSession,
    )

    parser = build_arg_parser()
    parser.add_argument("--ros-topic", default="/magloc/tracker_pose")
    parser.add_argument("--ros-frame", default="plane1")
    parser.add_argument("--ros-swap-xy", action="store_true")
    parser.add_argument("--ros-flip-x", action="store_true")
    parser.add_argument("--ros-flip-y", action="store_true")
    parser.add_argument("--ros-flip-z", action="store_true")
    parser.add_argument(
        "--ros-translation-m",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=(0.0, 0.0, 0.0),
        help="translation added after the axis mapping",
    )
    parser.add_argument(
        "--nucleo-reuse-sync",
        action="store_true",
        help="open an already-streaming board without sending another SYNC",
    )
    args = parser.parse_args(remaining)

    if not str(args.ros_topic).startswith("/"):
        parser.error("--ros-topic must be an absolute ROS topic")
    if not str(args.ros_frame).strip():
        parser.error("--ros-frame must not be empty")

    class HxNucleoSource(BaseSource):
        """Feed MagLoc with the project-validated hospital NUCLEO parser."""

        def __init__(self, cfg: Any) -> None:
            self._cfg = cfg
            self._ignored = {
                int(value) for value in cfg.get("source.ignored_sensor_ids", [])
            }
            self._sensor_array = load_sensor_array_csv(
                cfg.path("sensor_array"),
                ignored_sensor_ids=sorted(self._ignored),
            )
            self._sensor_count = len(self._sensor_array)
            self._parser = HubStreamParser(expected_channels=(1, 2, 3, 4))
            self._serial: Any | None = None

        def stream(self):
            import serial

            port = str(self._cfg.require("source.port"))
            baudrate = int(self._cfg.get("source.baudrate", 115200))
            reuse_sync = bool(self._cfg.get("source.nucleo_reuse_sync", False))
            self._serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.02,
                write_timeout=1.0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            time.sleep(0.2)
            initial_bytes = b""
            if reuse_sync:
                print("[HxNucleoSource] reusing existing NUCLEO SYNC", flush=True)
            else:
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
                helper = NucleoCaptureSession(
                    "/home/huaxin/Codes/ur-waypoint",
                    NucleoCaptureConfig(port=port, baudrate=baudrate),
                    time.time_ns,
                )
                sync_time, rtt_ns, attempts, initial_bytes = helper._synchronize(
                    self._serial
                )
                print(
                    "[HxNucleoSource] SYNC acknowledged "
                    f"sync_time={sync_time} attempts={attempts} "
                    f"rtt_ms={rtt_ns / 1e6:.3f}",
                    flush=True,
                )

            pending_frames = list(
                self._parser.feed(initial_bytes, time.monotonic_ns())
            )
            max_frames_raw = self._cfg.get("source.max_frames")
            max_frames = None if max_frames_raw is None else int(max_frames_raw)
            duration_raw = self._cfg.get("source.duration_s")
            duration_s = None if duration_raw is None else float(duration_raw)
            first_timestamp_s: float | None = None
            last_timestamp_s: float | None = None
            emitted = 0

            while max_frames is None or emitted < max_frames:
                if not pending_frames:
                    chunk = self._serial.read(4096)
                    now_ns = time.monotonic_ns()
                    if chunk:
                        pending_frames.extend(self._parser.feed(chunk, now_ns))
                    else:
                        self._parser.poll(now_ns)
                    if not pending_frames:
                        continue
                frame = pending_frames.pop(0)
                timestamp_s = float(frame.host_monotonic_ns) / 1e9
                if first_timestamp_s is None:
                    first_timestamp_s = timestamp_s
                if duration_s is not None and timestamp_s - first_timestamp_s > duration_s:
                    break

                magnetic_T = np.full((self._sensor_count, 3), np.nan, dtype=float)
                for sensor_id, xyz_ut in frame.sensor_values_ut.items():
                    if 0 <= sensor_id < self._sensor_count and sensor_id not in self._ignored:
                        magnetic_T[sensor_id] = np.asarray(xyz_ut, dtype=float) * 1e-6
                measurement = MeasurementFrame(
                    seq=int(frame.logical_seq),
                    timestamp_s=timestamp_s,
                    sensor_array=self._sensor_array,
                    magnetic_T=magnetic_T,
                )
                dt_s = (
                    0.0
                    if last_timestamp_s is None
                    else max(0.0, timestamp_s - last_timestamp_s)
                )
                last_timestamp_s = timestamp_s
                yield MeasurementPacket(
                    frame=measurement,
                    predict_dt_s=dt_s,
                    source_index=emitted,
                )
                emitted += 1

        def close(self) -> None:
            if self._serial is not None:
                self._serial.close()
                self._serial = None

    SourceRegistry.register("hx_nucleo", HxNucleoSource)

    class RosPoseSink(BaseSink):
        def __init__(self, cfg: Any) -> None:
            rclpy.init(args=[])
            self._node = rclpy.create_node("magloc_tracker_pose_publisher")
            self._publisher = self._node.create_publisher(
                PoseStamped,
                str(cfg.get("sink.ros_pose.topic")),
                qos_profile_sensor_data,
            )
            self._frame = str(cfg.get("sink.ros_pose.frame_id"))
            self._swap_xy = bool(cfg.get("sink.ros_pose.swap_xy", False))
            self._sign = np.asarray(
                [
                    -1.0 if cfg.get("sink.ros_pose.flip_x", False) else 1.0,
                    -1.0 if cfg.get("sink.ros_pose.flip_y", False) else 1.0,
                    -1.0 if cfg.get("sink.ros_pose.flip_z", False) else 1.0,
                ],
                dtype=float,
            )
            self._translation = np.asarray(
                cfg.get("sink.ros_pose.translation_m", [0.0, 0.0, 0.0]),
                dtype=float,
            ).reshape(3)
            self._node.get_logger().info(
                "publishing tracked positions on %s in frame %s; "
                "swap_xy=%s sign=%s translation_m=%s"
                % (
                    str(cfg.get("sink.ros_pose.topic")),
                    self._frame,
                    self._swap_xy,
                    self._sign.tolist(),
                    self._translation.tolist(),
                )
            )

        def write(self, result: Any) -> None:
            if not bool(result.update_applied):
                return
            position = np.asarray(result.estimate.state[POS_IDX], dtype=float).copy()
            if position.shape != (3,) or not np.all(np.isfinite(position)):
                return
            if self._swap_xy:
                position[[0, 1]] = position[[1, 0]]
            position *= self._sign
            position += self._translation

            message = PoseStamped()
            message.header.stamp = self._node.get_clock().now().to_msg()
            message.header.frame_id = self._frame
            message.pose.position.x = float(position[0])
            message.pose.position.y = float(position[1])
            message.pose.position.z = float(position[2])
            message.pose.orientation.w = 1.0
            self._publisher.publish(message)

        def close(self) -> None:
            self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    SinkRegistry.register("ros_pose", RosPoseSink)

    if args.numeric_threads is not None:
        os.environ[NUMERIC_THREAD_ENV_VAR] = str(args.numeric_threads)
    configure_numeric_threads()
    args.calibration = False

    overrides = _build_known_background_overrides(args)
    overrides.setdefault("source", {})
    overrides["source"]["kind"] = "hx_nucleo"
    overrides["source"]["nucleo_reuse_sync"] = bool(args.nucleo_reuse_sync)
    overrides.setdefault("sink", {})
    overrides["sink"]["kinds"] = ["evaluation", "ros_pose"]
    overrides["sink"]["ros_pose"] = {
        "topic": args.ros_topic,
        "frame_id": args.ros_frame,
        "swap_xy": bool(args.ros_swap_xy),
        "flip_x": bool(args.ros_flip_x),
        "flip_y": bool(args.ros_flip_y),
        "flip_z": bool(args.ros_flip_z),
        "translation_m": [float(value) for value in args.ros_translation_m],
    }
    cfg = config_use(
        args.profile,
        "real",
        overrides,
        source_file=magloc_root / "magloc" / "cli" / "track_known_background_dipole.py",
    )
    run_pipeline(cfg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
