from __future__ import annotations

import csv
import math
import threading
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

TRACKER_RAW_COLUMNS = (
    "tracker_rx_sync_time_ns",
    "tracker_rx_ros_time_ns",
    "tracker_message_ros_time_ns",
    "frame_id",
    "tracker_x_m",
    "tracker_y_m",
    "tracker_z_m",
    "tracker_qx",
    "tracker_qy",
    "tracker_qz",
    "tracker_qw",
)


class TrackerCaptureError(RuntimeError):
    """Raised before robot motion when a tracker stream is not trustworthy."""


@dataclass(frozen=True)
class TrackerCaptureConfig:
    topic: str
    frame_id: str = "plane1"
    preflight_timeout_s: float = 5.0
    max_mapping_offset_s: float = 0.05
    minimum_mapped_samples: int = 50

    def __post_init__(self) -> None:
        if (
            not isinstance(self.topic, str)
            or not self.topic.startswith("/")
            or any(character.isspace() for character in self.topic)
        ):
            raise ValueError("tracker topic must be an absolute ROS topic")
        if (
            not isinstance(self.frame_id, str)
            or not self.frame_id
            or self.frame_id != self.frame_id.strip()
            or any(character.isspace() for character in self.frame_id)
        ):
            raise ValueError("tracker frame_id must be a non-empty ROS frame")
        for value, label in (
            (self.preflight_timeout_s, "preflight_timeout_s"),
            (self.max_mapping_offset_s, "max_mapping_offset_s"),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"tracker {label} must be finite and positive")
        if (
            isinstance(self.minimum_mapped_samples, bool)
            or not isinstance(self.minimum_mapped_samples, int)
            or self.minimum_mapped_samples < 2
        ):
            raise ValueError("tracker minimum_mapped_samples must be an integer >= 2")


@dataclass(frozen=True)
class TrackerSample:
    tracker_rx_sync_time_ns: int
    tracker_rx_ros_time_ns: int
    tracker_message_ros_time_ns: int
    frame_id: str
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class TrackerCaptureSummary:
    config: TrackerCaptureConfig
    staging_path: Path
    samples: tuple[TrackerSample, ...]
    invalid_message_count: int
    started_sync_time_ns: int
    stopped_sync_time_ns: int


class TrackerCaptureSession:
    """Capture one PoseStamped stream using host monotonic receive timestamps."""

    def __init__(
        self,
        workspace: str | Path,
        node: Any,
        config: TrackerCaptureConfig,
    ) -> None:
        self._workspace = Path(workspace).expanduser().resolve()
        self._node = node
        self._config = config
        self._lock = threading.Lock()
        self._samples: list[TrackerSample] = []
        self._invalid_message_count = 0
        self._last_invalid_reason = ""
        self._subscription: Any | None = None
        self._active = False
        self._started_sync_time_ns = 0
        self._stopped_sync_time_ns = 0
        self._summary: TrackerCaptureSummary | None = None

    @property
    def config(self) -> TrackerCaptureConfig:
        return self._config

    def start(self) -> None:
        if self._active or self._summary is not None:
            raise TrackerCaptureError("tracker capture session cannot be started twice")
        runs_directory = self._workspace / "runs"
        if not runs_directory.is_dir():
            raise TrackerCaptureError(f"workspace runs directory does not exist: {runs_directory}")
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from rclpy.qos import qos_profile_sensor_data
        except ImportError as error:
            raise TrackerCaptureError(
                "ROS geometry_msgs/rclpy are required for tracker capture"
            ) from error

        self._active = True
        self._started_sync_time_ns = time.monotonic_ns()
        self._subscription = self._node.create_subscription(
            PoseStamped,
            self._config.topic,
            self._on_pose,
            qos_profile_sensor_data,
        )
        deadline = time.monotonic() + self._config.preflight_timeout_s
        publisher_probe_deadline: float | None = None
        while time.monotonic() < deadline:
            publisher_ready = self._pose_stamped_publisher_ready()
            if publisher_ready and publisher_probe_deadline is None:
                publisher_probe_deadline = min(deadline, time.monotonic() + 0.25)
            rclpy.spin_once(
                self._node,
                timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())),
            )
            with self._lock:
                if self._samples:
                    return
                invalid_count = self._invalid_message_count
                invalid_reason = self._last_invalid_reason
            if (
                publisher_probe_deadline is not None
                and time.monotonic() >= publisher_probe_deadline
            ):
                if invalid_count:
                    try:
                        self.stop()
                    except Exception:
                        pass
                    raise TrackerCaptureError(
                        f"tracker preflight rejected {self._config.topic}: {invalid_reason}"
                    )
                # The tracker may intentionally publish no pose until the warmup
                # motion has initialized tracking.  A correctly typed publisher
                # is enough to arm capture; finalization still requires complete
                # valid coverage of the yaw interval.
                return
        reason = self._last_invalid_reason or "no PoseStamped message was received"
        try:
            self.stop()
        except Exception:
            pass
        raise TrackerCaptureError(
            f"tracker preflight timed out on {self._config.topic}: {reason}"
        )

    def _pose_stamped_publisher_ready(self) -> bool:
        query = getattr(self._node, "get_publishers_info_by_topic", None)
        if not callable(query):
            return False
        try:
            endpoints = query(self._config.topic)
        except (RuntimeError, TypeError, ValueError):
            return False
        accepted_types = {
            "geometry_msgs/msg/PoseStamped",
            "geometry_msgs/PoseStamped",
        }
        return any(
            str(getattr(endpoint, "topic_type", "")) in accepted_types
            for endpoint in endpoints
        )

    def _on_pose(self, message: Any) -> None:
        if not self._active:
            return
        frame_id = str(message.header.frame_id)
        values = (
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
            float(message.pose.orientation.x),
            float(message.pose.orientation.y),
            float(message.pose.orientation.z),
            float(message.pose.orientation.w),
        )
        invalid_reason = ""
        if frame_id != self._config.frame_id:
            invalid_reason = (
                f"message frame_id {frame_id!r} does not equal required "
                f"{self._config.frame_id!r}"
            )
        elif not all(math.isfinite(value) for value in values):
            invalid_reason = "message pose contains non-finite values"
        else:
            quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
            if abs(quaternion_norm - 1.0) > 1.0e-3:
                invalid_reason = (
                    f"message quaternion is not normalized (norm={quaternion_norm:.8f})"
                )
        with self._lock:
            if invalid_reason:
                self._invalid_message_count += 1
                self._last_invalid_reason = invalid_reason
                return
            stamp = message.header.stamp
            self._samples.append(
                TrackerSample(
                    tracker_rx_sync_time_ns=time.monotonic_ns(),
                    tracker_rx_ros_time_ns=int(self._node.get_clock().now().nanoseconds),
                    tracker_message_ros_time_ns=(
                        int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                    ),
                    frame_id=frame_id,
                    position_m=(values[0], values[1], values[2]),
                    quaternion_xyzw=(values[3], values[4], values[5], values[6]),
                )
            )

    def stop(self) -> TrackerCaptureSummary:
        if self._summary is not None:
            return self._summary
        self._active = False
        self._stopped_sync_time_ns = time.monotonic_ns()
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None
        with self._lock:
            samples = tuple(self._samples)
            invalid_count = self._invalid_message_count
        staging_path = self._new_staging_path()
        self._write_staging(staging_path, samples)
        self._summary = TrackerCaptureSummary(
            config=self._config,
            staging_path=staging_path,
            samples=samples,
            invalid_message_count=invalid_count,
            started_sync_time_ns=self._started_sync_time_ns,
            stopped_sync_time_ns=self._stopped_sync_time_ns,
        )
        return self._summary

    def _new_staging_path(self) -> Path:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        nonce = uuid.uuid4().hex[:8]
        return self._workspace / "runs" / f"tracker_capture__{timestamp}__{nonce}.staging.csv"

    @staticmethod
    def _write_staging(path: Path, samples: tuple[TrackerSample, ...]) -> None:
        with path.open("x", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=TRACKER_RAW_COLUMNS)
            writer.writeheader()
            for sample in samples:
                writer.writerow(
                    {
                        "tracker_rx_sync_time_ns": sample.tracker_rx_sync_time_ns,
                        "tracker_rx_ros_time_ns": sample.tracker_rx_ros_time_ns,
                        "tracker_message_ros_time_ns": sample.tracker_message_ros_time_ns,
                        "frame_id": sample.frame_id,
                        "tracker_x_m": format(sample.position_m[0], ".17g"),
                        "tracker_y_m": format(sample.position_m[1], ".17g"),
                        "tracker_z_m": format(sample.position_m[2], ".17g"),
                        "tracker_qx": format(sample.quaternion_xyzw[0], ".17g"),
                        "tracker_qy": format(sample.quaternion_xyzw[1], ".17g"),
                        "tracker_qz": format(sample.quaternion_xyzw[2], ".17g"),
                        "tracker_qw": format(sample.quaternion_xyzw[3], ".17g"),
                    }
                )
