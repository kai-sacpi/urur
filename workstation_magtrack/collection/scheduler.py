"""Pure collection scheduling; callers execute actions and supply success evidence.

No ROS, serial, or motion code lives here. A trajectory is one indivisible action:
the scheduler never changes its points or timestamps. Checkpoints are written
before an action is returned, so interrupted physical actions are never replayed
automatically after loading a checkpoint. Use one runner/writer per checkpoint.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import errno
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Mapping, Sequence
from uuid import uuid4


class SchedulerError(RuntimeError):
    """A state, clock, or completion violates the orchestration contract."""


@dataclass(frozen=True)
class Action:
    action_id: str
    kind: str
    payload: dict[str, object]

    def to_dict(self) -> dict[str, object]:
        return {"action_id": self.action_id, "kind": self.kind,
                "payload": deepcopy(self.payload)}


def _nonempty(value: str, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a nonempty explicit string")
    return value


def _clock(value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError("now_ns must be a nonnegative monotonic integer")
    return value


def _seconds_ns(value: float, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be positive and finite")
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{field} must be positive and finite")
    result = round(value * 1_000_000_000)
    if result <= 0:
        raise ValueError(f"{field} is below clock resolution")
    return result


def _json_copy(value: Mapping[str, object]) -> dict[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError("metadata/evidence must be a mapping")
    if any(not isinstance(key, str) for key in value):
        raise ValueError("metadata/evidence keys must be strings")
    return json.loads(json.dumps(dict(value), allow_nan=False))


def _background_carryover(
    count: int, provenance: Mapping[str, object] | None, *,
    hardware_boot_id: str, pause_after: int,
) -> dict[str, object] | None:
    """Validate a caller-verified, immutable proof of backgrounds in this boot.

    File existence and digest verification belong to the hardware-free runner
    preflight. The scheduler only retains their binding and boot/count identity.
    """
    if (isinstance(count, bool) or not isinstance(count, int)
            or not 0 <= count < pause_after):
        raise ValueError("initial_prior_background_count must be an integer in [0, pause_after)")
    if count == 0:
        if provenance is not None:
            raise ValueError("zero prior backgrounds must not have carryover provenance")
        return None
    if provenance is None:
        raise ValueError("prior backgrounds require immutable carryover provenance")
    proof = _json_copy(provenance)
    proof_count = proof.get("accepted_background_count")
    if (isinstance(proof_count, bool) or not isinstance(proof_count, int)
            or proof_count != count or proof.get("hardware_boot_id") != hardware_boot_id):
        raise ValueError("carryover provenance must match the current hardware boot and count")
    _nonempty(proof.get("evidence_path"), "carryover evidence_path")
    digest = proof.get("evidence_sha256")
    if (not isinstance(digest, str) or len(digest) != 64
            or any(char not in "0123456789abcdef" for char in digest)):
        raise ValueError("carryover evidence_sha256 must be a lowercase SHA-256 digest")
    return proof


class Engine:
    """Single-writer state machine with durable, inspectable JSON checkpoints."""

    SCHEMA_VERSION = 1
    STATUSES = {"running", "waiting_for_nucleo_restart", "complete", "needs_attention"}
    STAGES = {"opening", "boundary", "background_home", "background_park",
              "background_capture", "background_return_home", "pause_close",
              "final_home", "final_close", "paused", "finished"}

    def __init__(self, path: Path, state: dict[str, object]) -> None:
        self.path = Path(path)
        self._state = state

    @classmethod
    def create(
        cls, checkpoint_path: Path | str, trajectory_ids: Sequence[str],
        hardware_boot_id: str, now_ns: int, *, clock_epoch_id: str,
        interval_s: float = 600, capture_s: float = 30, pause_after: int = 3,
        metadata: Mapping[str, object] | None = None,
        initial_background: bool = False,
        initial_prior_background_count: int = 0,
        prior_background_provenance: Mapping[str, object] | None = None,
    ) -> Engine:
        """Start an explicitly named hardware boot; do not open the hardware."""
        if isinstance(trajectory_ids, (str, bytes)):
            raise ValueError("trajectory_ids must be a nonempty sequence of unique IDs")
        ids = [_nonempty(item, "trajectory ID") for item in trajectory_ids]
        if not ids or len(set(ids)) != len(ids):
            raise ValueError("trajectory_ids must be nonempty and unique")
        if isinstance(pause_after, bool) or not isinstance(pause_after, int) or pause_after < 1:
            raise ValueError("pause_after must be a positive integer")
        if not isinstance(initial_background, bool):
            raise ValueError("initial_background must be a boolean")
        hardware_boot_id = _nonempty(hardware_boot_id, "hardware_boot_id")
        carryover = _background_carryover(
            initial_prior_background_count, prior_background_provenance,
            hardware_boot_id=hardware_boot_id, pause_after=pause_after)
        now_ns = _clock(now_ns)
        state: dict[str, object] = {
            "schema_version": cls.SCHEMA_VERSION,
            "campaign_id": uuid4().hex,
            "metadata": _json_copy(metadata if metadata is not None else {}),
            "trajectory_ids": ids, "next_cursor": 0,
            "interval_ns": _seconds_ns(interval_s, "interval_s"),
            "capture_ns": _seconds_ns(capture_s, "capture_s"),
            "pause_after": pause_after,
            "clock_epoch_id": _nonempty(clock_epoch_id, "clock_epoch_id"),
            "last_clock_ns": now_ns,
            "hardware_boot_ids": [hardware_boot_id],
            "hardware_boot_id": hardware_boot_id,
            "session_id": uuid4().hex, "session_number": 1,
            "session_open": False, "session_started_ns": None,
            "session_background_count": initial_prior_background_count,
            "total_background_count": initial_prior_background_count,
            "initial_background_carryover": carryover,
            "last_capture_end_ns": None, "next_due_ns": None,
            "initial_background_pending": initial_background,
            "status": "running", "stage": "opening", "active_action": None,
            "active_action_started_ns": None,
            "attention_reason": None, "events": [],
        }
        engine = cls(Path(checkpoint_path), state)
        engine._event("created", now_ns)
        if carryover is not None:
            engine._event("prior_backgrounds_carried_over", now_ns,
                          accepted_background_count=initial_prior_background_count,
                          provenance=deepcopy(carryover))
        engine._save(create=True)
        return engine

    @classmethod
    def load(
        cls, checkpoint_path: Path | str, *, clock_epoch_id: str,
        expected_metadata: Mapping[str, object] | None = None,
    ) -> Engine:
        """Reload idle state only; an interrupted physical action needs attention."""
        path = Path(checkpoint_path)
        with path.open(encoding="utf-8") as stream:
            state = json.load(stream)
        cls._validate_state(state)
        if expected_metadata is not None and state["metadata"] != _json_copy(expected_metadata):
            raise SchedulerError("checkpoint metadata does not match current manifest/config")
        engine = cls(path, state)
        reasons = []
        if state["clock_epoch_id"] != _nonempty(clock_epoch_id, "clock_epoch_id"):
            reasons.append("clock_epoch_changed; old monotonic timestamps are not reusable")
        if state["active_action"] is not None:
            reasons.append("interrupted_in_progress_action; manual physical-state review required")
        if reasons:
            engine._attention("; ".join(reasons), int(state["last_clock_ns"]))
            engine._save()
        return engine

    @classmethod
    def _validate_state(cls, state: object) -> None:
        if not isinstance(state, dict) or state.get("schema_version") != cls.SCHEMA_VERSION:
            raise SchedulerError("unsupported or invalid checkpoint schema")
        if state.get("status") not in cls.STATUSES or state.get("stage") not in cls.STAGES:
            raise SchedulerError("invalid checkpoint state")
        ids, cursor = state.get("trajectory_ids"), state.get("next_cursor")
        if (not isinstance(ids, list) or not ids or any(not isinstance(x, str) or not x for x in ids)
                or len(set(ids)) != len(ids) or isinstance(cursor, bool)
                or not isinstance(cursor, int) or not 0 <= cursor <= len(ids)):
            raise SchedulerError("invalid checkpoint trajectory queue/cursor")
        for key in ("interval_ns", "capture_ns", "pause_after", "session_number"):
            if isinstance(state.get(key), bool) or not isinstance(state.get(key), int) or state[key] < 1:
                raise SchedulerError(f"invalid checkpoint {key}")
        for key in ("last_clock_ns", "session_background_count", "total_background_count"):
            if isinstance(state.get(key), bool) or not isinstance(state.get(key), int) or state[key] < 0:
                raise SchedulerError(f"invalid checkpoint {key}")
        for key in ("session_started_ns", "last_capture_end_ns", "next_due_ns"):
            value = state.get(key)
            if value is not None and (isinstance(value, bool) or not isinstance(value, int) or value < 0):
                raise SchedulerError(f"invalid checkpoint {key}")
        for key in ("hardware_boot_id", "session_id", "clock_epoch_id", "campaign_id"):
            if not isinstance(state.get(key), str) or not state[key].strip():
                raise SchedulerError(f"invalid checkpoint {key}")
        if not isinstance(state.get("session_open"), bool) or not isinstance(state.get("events"), list):
            raise SchedulerError("invalid checkpoint session/events")
        if not isinstance(state.get("metadata"), dict):
            raise SchedulerError("invalid checkpoint metadata")
        if not isinstance(state.get("initial_background_pending", False), bool):
            raise SchedulerError("invalid checkpoint initial_background_pending")
        boots = state.get("hardware_boot_ids")
        if (not isinstance(boots, list) or any(not isinstance(x, str) or not x for x in boots)
                or len(set(boots)) != len(boots) or state["hardware_boot_id"] not in boots):
            raise SchedulerError("invalid checkpoint hardware boot history")
        active = state.get("active_action")
        if active is not None and (not isinstance(active, dict)
                                  or not all(key in active for key in ("action_id", "kind", "payload"))):
            raise SchedulerError("invalid checkpoint active action")
        if state["status"] == "waiting_for_nucleo_restart" and (
                state["stage"] != "paused" or state["session_open"] or active is not None):
            raise SchedulerError("paused checkpoint must be home, closed, and idle")
        if state["status"] == "complete" and (
                state["stage"] != "finished" or state["session_open"] or active is not None
                or cursor != len(ids)):
            raise SchedulerError("invalid completed checkpoint")
        if state["session_background_count"] > state["total_background_count"]:
            raise SchedulerError("invalid checkpoint background counts")
        carryover = state.get("initial_background_carryover")
        if carryover is not None:
            if not isinstance(carryover, dict):
                raise SchedulerError("invalid checkpoint background carryover provenance")
            try:
                prior_count = carryover.get("accepted_background_count")
                _background_carryover(prior_count, carryover,
                                      hardware_boot_id=boots[0],
                                      pause_after=state["pause_after"])
            except (TypeError, ValueError) as exc:
                raise SchedulerError("invalid checkpoint background carryover provenance") from exc
            if (state["total_background_count"] < prior_count
                    or (state["session_number"] == 1
                        and state["session_background_count"] < prior_count)):
                raise SchedulerError("checkpoint background counts lost the accepted carryover")
        if state["status"] == "running":
            if state["stage"] in ("paused", "finished"):
                raise SchedulerError("running checkpoint cannot be paused/finished")
            if state["stage"] == "opening":
                if state["session_open"] or state["next_due_ns"] is not None:
                    raise SchedulerError("opening checkpoint cannot reuse an existing sensor session")
            elif not state["session_open"] or state["next_due_ns"] is None:
                raise SchedulerError("running actions require an open sensor session and deadline")
        issued_ns = state.get("active_action_started_ns")
        if (active is None and issued_ns is not None) or (active is not None and (
                isinstance(issued_ns, bool) or not isinstance(issued_ns, int) or issued_ns < 0)):
            raise SchedulerError("invalid checkpoint active action timestamp")
        try:
            json.dumps(state, allow_nan=False)
        except (ValueError, TypeError) as exc:
            raise SchedulerError("checkpoint contains non-JSON or nonfinite values") from exc

    @property
    def snapshot(self) -> dict[str, object]:
        return deepcopy(self._state)

    @property
    def status(self) -> str:
        return str(self._state["status"])

    @property
    def next_cursor(self) -> int:
        return int(self._state["next_cursor"])

    def _event(self, event: str, now_ns: int, **detail: object) -> None:
        self._state["events"].append({"event": event, "monotonic_ns": now_ns,
                                      "session_id": self._state["session_id"], **detail})

    def _attention(self, reason: str, now_ns: int) -> None:
        self._state["status"] = "needs_attention"
        self._state["attention_reason"] = reason
        self._event("needs_attention", now_ns, reason=reason)

    def _observe_clock(self, now_ns: int) -> int:
        now_ns = _clock(now_ns)
        if now_ns < self._state["last_clock_ns"]:
            self._attention("monotonic_clock_moved_backwards", int(self._state["last_clock_ns"]))
            self._save()
            raise SchedulerError("monotonic clock moved backwards; manual review required")
        self._state["last_clock_ns"] = now_ns
        return now_ns

    def take_next(self, now_ns: int) -> Action | None:
        """Persist and return one action, never returning an in-progress action twice."""
        if self.status != "running":
            return None
        if self._state["active_action"] is not None:
            raise SchedulerError("an action is already in progress")
        now_ns = self._observe_clock(now_ns)
        stage = self._state["stage"]
        if stage == "boundary":
            if self.next_cursor == len(self._state["trajectory_ids"]):
                stage = "final_home"
            elif self._state["next_due_ns"] is not None and now_ns >= self._state["next_due_ns"]:
                stage = "background_home"
                self._event("background_due_at_trajectory_boundary", now_ns,
                            due_ns=self._state["next_due_ns"],
                            boundary_lateness_ns=now_ns - self._state["next_due_ns"])
            self._state["stage"] = stage
        payload: dict[str, object] = {
            "session_id": self._state["session_id"],
            "hardware_boot_id": self._state["hardware_boot_id"],
            "clock_epoch_id": self._state["clock_epoch_id"],
        }
        if stage == "opening":
            kind = "open_sensor_session"
        elif stage == "boundary":
            kind = "play_trajectory"
            payload.update(trajectory_id=self._state["trajectory_ids"][self.next_cursor],
                           trajectory_index=self.next_cursor)
        elif stage in ("background_home", "background_return_home", "final_home"):
            kind = "move_home"
            payload["purpose"] = stage
        elif stage == "background_park":
            kind = "move_park"
        elif stage == "background_capture":
            kind = "capture_background"
            payload.update(duration_s=self._state["capture_ns"] / 1_000_000_000,
                           background_ordinal=self._state["total_background_count"] + 1,
                           session_background_ordinal=self._state["session_background_count"] + 1)
        elif stage in ("pause_close", "final_close"):
            kind = "close_sensor_session"
            payload["reason"] = "nucleo_restart_required" if stage == "pause_close" else "campaign_finished"
        else:
            raise SchedulerError(f"illegal running stage: {stage}")
        action = Action(uuid4().hex, kind, payload)
        self._state["active_action"] = action.to_dict()
        self._state["active_action_started_ns"] = now_ns
        self._event("action_started", now_ns, action=action.to_dict())
        self._save()
        return action

    def complete(self, action_id: str, evidence: Mapping[str, object], now_ns: int) -> None:
        """Acknowledge a successfully completed action, with adapter-owned evidence."""
        action = self._require_action(action_id)
        evidence_copy = _json_copy(evidence)
        now_ns = self._observe_clock(now_ns)
        if (self._state["stage"] == "background_capture"
                and now_ns - self._state["active_action_started_ns"] < self._state["capture_ns"]):
            raise SchedulerError("background capture must finish its entire configured duration")
        stage = self._state["stage"]
        self._event("action_completed", now_ns, action_id=action_id,
                    kind=action["kind"], evidence=evidence_copy)
        if stage == "opening":
            initial_background = self._state.get("initial_background_pending", False)
            self._state.update(session_open=True, session_started_ns=now_ns,
                               next_due_ns=now_ns + self._state["interval_ns"],
                               initial_background_pending=False,
                               stage="background_home" if initial_background else "boundary")
            if initial_background:
                self._event("initial_background_requested", now_ns)
        elif stage == "boundary":
            self._state["next_cursor"] += 1
        elif stage == "background_home":
            self._state["stage"] = "background_park"
        elif stage == "background_park":
            self._state["stage"] = "background_capture"
        elif stage == "background_capture":
            self._state["total_background_count"] += 1
            self._state["session_background_count"] += 1
            self._state.update(last_capture_end_ns=now_ns,
                               next_due_ns=now_ns + self._state["interval_ns"],
                               stage="background_return_home")
        elif stage == "background_return_home":
            self._state["stage"] = ("pause_close" if self._state["session_background_count"]
                                     >= self._state["pause_after"] else "boundary")
        elif stage == "pause_close":
            self._state.update(session_open=False, status="waiting_for_nucleo_restart", stage="paused")
            self._event("waiting_for_nucleo_restart", now_ns, next_cursor=self.next_cursor)
        elif stage == "final_home":
            self._state["stage"] = "final_close"
        elif stage == "final_close":
            self._state.update(session_open=False, status="complete", stage="finished")
        else:
            raise SchedulerError(f"illegal completion stage: {stage}")
        self._state["active_action"] = None
        self._state["active_action_started_ns"] = None
        self._save()

    def _require_action(self, action_id: str) -> dict[str, object]:
        active = self._state["active_action"]
        if self.status != "running" or active is None or active["action_id"] != action_id:
            raise SchedulerError("no matching in-progress action (duplicate/stale completion refused)")
        return active

    def fail(self, action_id: str, error: str, now_ns: int) -> None:
        """Stop dispatching; never assume returning home is safe after a failure."""
        action = self._require_action(action_id)
        error = _nonempty(error, "error")
        now_ns = self._observe_clock(now_ns)
        self._event("action_failed", now_ns, action_id=action_id, kind=action["kind"], error=error)
        self._state["active_action"] = None
        self._state["active_action_started_ns"] = None
        self._attention(error, now_ns)
        self._save()

    def confirm_restart(self, new_hardware_boot_id: str, now_ns: int) -> None:
        """Only explicit confirmation of a never-used hardware boot resumes dispatch."""
        if self.status != "waiting_for_nucleo_restart":
            raise SchedulerError("restart confirmation is allowed only at the closed home pause")
        boot_id = _nonempty(new_hardware_boot_id, "new_hardware_boot_id")
        if boot_id in self._state["hardware_boot_ids"]:
            raise SchedulerError("new hardware boot ID must never have been used in this campaign")
        now_ns = self._observe_clock(now_ns)
        self._state["hardware_boot_ids"].append(boot_id)
        self._state.update(hardware_boot_id=boot_id, session_id=uuid4().hex,
                           session_number=self._state["session_number"] + 1,
                           session_background_count=0, session_started_ns=None,
                           last_capture_end_ns=None, next_due_ns=None,
                           status="running", stage="opening")
        self._event("hardware_restart_explicitly_confirmed", now_ns,
                    hardware_boot_id=boot_id, next_cursor=self.next_cursor)
        self._save()

    def _save(self, *, create: bool = False) -> None:
        try:
            self._write_checkpoint(create=create)
        except Exception as exc:
            # An adapter must never continue physical dispatch with an uncertain
            # completion checkpoint, even if its caller catches the I/O error.
            self._attention(f"checkpoint_write_failed: {exc}", int(self._state["last_clock_ns"]))
            raise

    def _write_checkpoint(self, *, create: bool = False) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        serialized = json.dumps(self._state, indent=2, sort_keys=True, allow_nan=False) + "\n"
        fd, temporary = tempfile.mkstemp(prefix=f".{self.path.name}.", suffix=".tmp", dir=self.path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                stream.write(serialized)
                stream.flush()
                os.fsync(stream.fileno())
            if create:
                os.link(temporary, self.path)  # Never overwrite an existing campaign.
                os.unlink(temporary)
            else:
                os.replace(temporary, self.path)
            directory_fd = os.open(self.path.parent, os.O_RDONLY)
            try:
                try:
                    os.fsync(directory_fd)
                except OSError as exc:
                    if exc.errno not in (errno.EINVAL, errno.ENOTSUP):
                        raise
            finally:
                os.close(directory_fd)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
