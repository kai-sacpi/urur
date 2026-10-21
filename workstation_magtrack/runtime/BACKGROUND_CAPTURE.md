# Continuous background acquisition

This entry point records empty fields without starting ROS or commanding the UR.
One NUCLEO connection is held for the full batch. It passively checks for an
existing synchronized stream; only a completely silent board receives one
initial SYNC transmission. A partial/invalid stream causes a stop instead of
another handshake. The first five seconds are a fixed startup warm-up, preserved
in the full-session raw CSV.

Each nonoverlapping 120-second host-monotonic window is recorded without
resampling, interpolation or synthetic samples. Raw rows and cumulative parser
counters are copied verbatim from the full-session stream. Each per-window
manifest separately records counter baselines, ending snapshots and their
deltas; a defect in an earlier window is not incorrectly charged to later ones.

The acceptance policy scales the earlier 30-second selection to 120 seconds:
at least 11940 complete four-channel groups, incomplete groups at most 0.5%,
actual sample coverage at least 119.9 seconds, and zero malformed, discarded,
source-drop, reordered, duplicate, timeout, eviction or unexpected-channel
counters within the window. This assesses capture integrity, not environmental
magnetic-field noise or stationarity. Rejected raw data are kept separately for
audit, never converted into accepted data.

The default target is 100 accepted windows. Rejected windows are replaced by new
120-second recordings, up to 150 total attempts. Five consecutive rejected
windows or a stream interruption stops the batch with `needs_attention` rather
than relaxing quality thresholds or repeatedly handshaking. At least 3 h 20 min
of recording is needed; each replacement adds 120 seconds.

On Ubuntu24, use the parent's existing Python environment:

```bash
cd /home/huaxin/Codes/ur-waypoint
.venv/bin/python -u workstation_magtrack/runtime/capture_background_batch.py \
  --port /dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3879336C3333-if00 \
  --output /home/huaxin/Codes/ur-waypoint/workstation_magtrack/backgrounds/UNIQUE_BATCH_NAME \
  --count 100 --duration-s 120
```

The output directory must be new. Progress is atomically saved to `status.json`.
Accepted records are in `accepted/`, rejected records in `rejected/`, and a
currently open record in `in_progress/`. Every finished record includes the
raw CSV, capture manifest and SHA-256 checksums. `attempts.jsonl` is the batch
index. `session/` retains the uninterrupted source CSV and final capture summary.
Capture files belong on Ubuntu24, under `workstation_magtrack/backgrounds/`.
