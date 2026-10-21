"""Shell sequencing checks with fake executables; no ROS, GPU, or remote I/O."""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import pytest

from workstation_magtrack.ops import export_overnight_delivery as delivery


@pytest.fixture
def fake_delivery(tmp_path: Path) -> tuple[Path, dict[str, str], Path]:
    parent = tmp_path / "repo"
    workspace = parent / "workstation_magtrack"
    ops = workspace / "ops"
    ops.mkdir(parents=True)
    script = ops / "finish_parking_delivery.sh"
    shutil.copyfile(Path(__file__).parents[1] / "ops/finish_parking_delivery.sh", script)
    binary = tmp_path / "bin"
    binary.mkdir()
    python = parent / ".venv/bin/python"
    python.parent.mkdir(parents=True)
    stubs = {
        binary / "uname": '#!/bin/sh\nprintf "Linux\\n"\n',
        binary / "bash": '#!/bin/sh\nprintf "BASH %s\\n" "$*" >> "$CALL_LOG"\n',
        python: ('#!/bin/sh\nprintf "PYTHON %s\\n" "$*" >> "$CALL_LOG"\n'
                 'case "$*" in *collection.prepared_parking*) exit "${FAIL_CREATE:-0}" ;; esac\n'),
    }
    for path, content in stubs.items():
        path.write_text(content)
        path.chmod(0o755)
    log = tmp_path / "calls.log"
    return script, {**os.environ, "PATH": f"{binary}:{os.environ['PATH']}",
                    "CALL_LOG": str(log)}, log


def test_offline_delivery_chain_uses_exact_order_and_no_replacement_producer(fake_delivery) -> None:
    script, env, log = fake_delivery
    # This is an actually live local PID, but fake executables only log its
    # forwarding; the shell must never signal it or launch another producer.
    producer_pid = str(os.getpid())
    result = subprocess.run(["/bin/bash", str(script), producer_pid], env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    calls = log.read_text()
    workspace = script.parents[1]
    validation = workspace / delivery.PARKING_VALIDATION
    bundle = workspace / delivery.PARKING_BUNDLE
    mock_output = (workspace / delivery.PARKING_MOCK).parent
    assert "wait_for_validation(Path(sys.argv[1]), wait=True, producer_pid=int(sys.argv[2]))" in calls
    assert f"{validation} {producer_pid}" in calls
    ordered = [calls.index(x) for x in ("PYTHON -c", "PYTHON -m collection.prepared_parking create",
                                       "BASH collection/run_mock_parking.sh", "BASH ops/overnight_delivery.sh --once")]
    assert ordered == sorted(ordered)
    assert calls.count("PYTHON ") == 2
    create_call = next(line for line in calls.splitlines() if line.startswith("PYTHON -m collection.prepared_parking create"))
    assert f"--validation {validation}" in create_call
    assert f"--producer-code-root {validation.parent}" in create_call
    assert f"--output {bundle}" in create_call
    mock_call = next(line for line in calls.splitlines() if line.startswith("BASH collection/run_mock_parking.sh"))
    assert f"--validation {validation}" in mock_call
    assert f"--bundle {bundle}" in mock_call
    assert f"--output {mock_output}" in mock_call
    assert "run_campaign.py" not in calls
    assert "collection.validate_parking" not in calls
    assert "validate_parking.py" not in calls
    assert "--gpu" not in calls


def test_offline_delivery_chain_stops_if_bundle_fails(fake_delivery) -> None:
    script, env, log = fake_delivery
    result = subprocess.run(["/bin/bash", str(script), "705832"], env={**env, "FAIL_CREATE": "17"},
                            capture_output=True, text=True)
    assert result.returncode == 17
    calls = log.read_text()
    assert "collection.prepared_parking create" in calls
    assert "BASH " not in calls


def test_offline_delivery_sigterm_reaches_owned_child_before_exit(fake_delivery) -> None:
    script, env, log = fake_delivery
    bash = Path(env["PATH"].split(":")[0]) / "bash"
    bash.write_text(f"#!{sys.executable}\n" + '''
import os
from pathlib import Path
import signal
import time
log = Path(os.environ["CALL_LOG"])
def terminated(*args):
    with log.open("a") as stream:
        stream.write("OWNED_CHILD_TERM_RECEIVED\\n")
    raise SystemExit(0)
signal.signal(signal.SIGTERM, terminated)
with log.open("a") as stream:
    stream.write("OWNED_CHILD_READY\\n")
while True:
    time.sleep(.01)
''')
    process = subprocess.Popen(["/bin/bash", str(script), "705832"], env=env,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        deadline = time.monotonic() + 5
        while not log.exists() or "OWNED_CHILD_READY" not in log.read_text():
            assert time.monotonic() < deadline, "fake child did not become ready"
            assert process.poll() is None
            time.sleep(.01)
        process.send_signal(signal.SIGTERM)
        process.communicate(timeout=5)
        assert process.returncode == 130
        assert "OWNED_CHILD_TERM_RECEIVED" in log.read_text()
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
            process.communicate(timeout=5)


@pytest.mark.parametrize("args", [[], ["0"], ["-1"], ["not-a-pid"], ["705832", "extra"]])
def test_offline_delivery_chain_rejects_bad_producer_without_actions(fake_delivery, args) -> None:
    script, env, log = fake_delivery
    result = subprocess.run(["/bin/bash", str(script), *args], env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert not log.exists()
