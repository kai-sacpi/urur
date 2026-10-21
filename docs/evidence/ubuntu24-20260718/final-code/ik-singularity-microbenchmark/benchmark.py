from __future__ import annotations

import hashlib
import inspect
import json
import os
import statistics
import time
from pathlib import Path

import numpy as np

from hx_waypoint_curobo import curobo_adapter_v2 as adapter_module
from hx_waypoint_curobo.curobo_adapter_v2 import CuroboAdapterV2
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder

RUN_DIR = Path(os.environ["RUN_DIR"]).resolve()
ROOT = Path("/home/huaxin/Codes/ur-waypoint").resolve()
COUNTS = (128, 512, 2048)
REPETITIONS = 3
SEED = 20260719


def synchronize(torch) -> None:
    torch.cuda.synchronize()


def timed(torch, function, repetitions: int):
    durations = []
    output = None
    for _ in range(repetitions):
        synchronize(torch)
        started = time.perf_counter()
        output = function()
        synchronize(torch)
        durations.append(time.perf_counter() - started)
    assert output is not None
    return output, durations


environment = EnvironmentBuilder().load(
    ROOT / "environments" / "legacy_plane1_test_p10_rod120"
)
adapter = CuroboAdapterV2(environment, ROOT / "curobo.lock")
runtime = adapter._ensure_runtime()
torch = runtime["torch"]
InverseKinematics = runtime["InverseKinematics"]
InverseKinematicsCfg = runtime["InverseKinematicsCfg"]
JointState = runtime["JointState"]

if adapter_module._SINGULARITY_SCORE_CHUNK_SIZE != 2048:
    raise RuntimeError(
        "unexpected singularity chunk size: "
        f"{adapter_module._SINGULARITY_SCORE_CHUNK_SIZE}"
    )

config = InverseKinematicsCfg.create(
    robot=adapter._robot_config(),
    scene_model=adapter._scene_config(),
    num_seeds=32,
    max_batch_size=1,
    self_collision_check=True,
    position_tolerance=float(
        adapter._planner_value("validation.tip_position_tolerance_m")
    ),
    orientation_tolerance=float(
        np.radians(
            adapter._planner_value("validation.tip_orientation_tolerance_deg")
        )
    ),
    optimizer_collision_activation_distance=float(
        adapter._planner_value("validation.minimum_collision_distance_m")
    ),
    use_cuda_graph=False,
)
solver = InverseKinematics(config)

home = np.asarray(
    environment.robot_model["robot"]["home_joint_positions"],
    dtype=np.float32,
)
rng = np.random.default_rng(SEED)
fixed_candidates = home + rng.uniform(
    -0.25,
    0.25,
    size=(max(COUNTS), len(environment.joint_names)),
).astype(np.float32)
np.save(RUN_DIR / "fixed_candidates.npy", fixed_candidates, allow_pickle=False)


def scalar_scores(positions: np.ndarray) -> np.ndarray:
    scores = []
    for positions_row in positions:
        state = JointState.from_position(
            torch.as_tensor(
                np.asarray(positions_row, dtype=np.float32).reshape(1, -1),
                device="cuda",
                dtype=torch.float32,
            ),
            joint_names=list(environment.joint_names),
        )
        kinematics = solver.compute_kinematics(state)
        jacobians = kinematics.tool_jacobians
        if jacobians is None:
            raise RuntimeError("scalar reference received no tool Jacobian")
        matrix = jacobians.reshape(-1, 6, len(environment.joint_names))[0]
        singular_values = torch.linalg.svdvals(matrix)
        minimum = float(torch.min(singular_values).item())
        maximum = float(torch.max(singular_values).item())
        scores.append(maximum / max(minimum, 1.0e-6))
    return np.asarray(scores, dtype=float)


# Warm both paths without including initialization in the measurements.
scalar_scores(fixed_candidates[:16])
adapter._candidate_singularity_scores(solver, fixed_candidates[:128])
synchronize(torch)

results = []
for count in COUNTS:
    candidates = fixed_candidates[:count]
    scalar, scalar_durations = timed(
        torch,
        lambda candidates=candidates: scalar_scores(candidates),
        REPETITIONS,
    )
    batched, batched_durations = timed(
        torch,
        lambda candidates=candidates: adapter._candidate_singularity_scores(
            solver, candidates
        ),
        REPETITIONS,
    )
    if scalar.shape != (count,) or batched.shape != (count,):
        raise RuntimeError("singularity score output shape mismatch")
    absolute_error = np.abs(scalar - batched)
    relative_error = absolute_error / np.maximum(np.abs(scalar), 1.0e-12)
    exact_equal = bool(np.array_equal(scalar, batched))
    allclose = bool(np.allclose(scalar, batched, rtol=1.0e-6, atol=1.0e-9))
    if not allclose:
        raise RuntimeError(f"batched score mismatch for candidate_count={count}")
    scalar_median = float(statistics.median(scalar_durations))
    batched_median = float(statistics.median(batched_durations))
    row = {
        "candidate_count": count,
        "candidate_prefix_sha256": hashlib.sha256(
            candidates.tobytes(order="C")
        ).hexdigest(),
        "scalar_seconds": [float(value) for value in scalar_durations],
        "batched_seconds": [float(value) for value in batched_durations],
        "scalar_median_seconds": scalar_median,
        "batched_median_seconds": batched_median,
        "speedup": scalar_median / batched_median,
        "exact_equal": exact_equal,
        "max_absolute_error": float(np.max(absolute_error, initial=0.0)),
        "max_relative_error": float(np.max(relative_error, initial=0.0)),
    }
    results.append(row)
    print(json.dumps(row, sort_keys=True))

report = {
    "schema_version": 1,
    "scope": "fixed-candidate batched-vs-scalar singularity microbenchmark only",
    "trajectory_files_opened": False,
    "environment_id": environment.environment_id,
    "candidate_seed": SEED,
    "candidate_dtype": str(fixed_candidates.dtype),
    "candidate_shape": list(fixed_candidates.shape),
    "joint_names": list(environment.joint_names),
    "chunk_size": adapter_module._SINGULARITY_SCORE_CHUNK_SIZE,
    "repetitions": REPETITIONS,
    "python_module_path": str(Path(inspect.getfile(adapter_module)).resolve()),
    "torch_version": torch.__version__,
    "torch_cuda_version": str(torch.version.cuda),
    "gpu_name": torch.cuda.get_device_name(0),
    "results": results,
    "passed": True,
}
(RUN_DIR / "report.json").write_text(
    json.dumps(report, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
