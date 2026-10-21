import hashlib
from pathlib import Path

from workstation_magtrack.ops.export_replay_index import check_seal


def test_missing_and_tampered_recording_are_not_marked_ready(tmp_path: Path) -> None:
    assert check_seal(tmp_path / "missing", "abc")["status"] == "not_available_on_this_host"
    recording = tmp_path / "recording"
    recording.mkdir()
    source = recording / "source_tip_trajectory.csv"
    source.write_bytes(b"authoritative source\n")
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    (recording / "sha256sums.txt").write_text(f"{digest}  source_tip_trajectory.csv\n")
    assert check_seal(recording, digest)["status"] == "passed"
    source.write_bytes(b"changed\n")
    assert check_seal(recording, digest)["status"] == "failed"


def test_checksum_list_cannot_escape_recording(tmp_path: Path) -> None:
    source = tmp_path / "source_tip_trajectory.csv"
    source.write_bytes(b"authority\n")
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    (tmp_path / "sha256sums.txt").write_text(f"{digest}  ../source_tip_trajectory.csv\n")
    assert check_seal(tmp_path, digest)["status"] == "failed"
