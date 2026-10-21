from __future__ import annotations

import signal
from types import SimpleNamespace

import pytest
from hx_waypoint_curobo.errors import WaypointError

from hx_waypoint_tools import cli


def test_simulation_launch_command_binds_selected_environment(
    monkeypatch, environment
) -> None:
    monkeypatch.setattr(
        cli.shutil, "which", lambda executable: "/opt/ros/jazzy/bin/ros2"
    )

    command = cli._simulation_launch_command(
        environment, environment.root.parent.parent, True
    )

    assert command[:4] == [
        "/opt/ros/jazzy/bin/ros2",
        "launch",
        "hx_waypoint_bringup",
        "simulation.launch.py",
    ]
    assert "ur_type:=ur5" in command
    assert "environment_id:=lab_wall_a" in command
    assert "launch_rviz:=true" in command


def test_simulation_launch_command_requires_ros2(monkeypatch, environment) -> None:
    monkeypatch.setattr(cli.shutil, "which", lambda executable: None)

    with pytest.raises(WaypointError, match="ros2 executable"):
        cli._simulation_launch_command(
            environment, environment.root.parent.parent, False
        )


def test_wait_for_build_server_rejects_early_launch_exit() -> None:
    client = SimpleNamespace(wait_for_server=lambda timeout_sec: False)
    process = SimpleNamespace(poll=lambda: 7)

    with pytest.raises(WaypointError, match="exit=7"):
        cli._wait_for_build_server(client, process, 1.0)


def test_managed_launch_is_stopped_as_a_process_group(monkeypatch) -> None:
    signals: list[tuple[int, int]] = []

    class Process:
        pid = 12345

        def __init__(self) -> None:
            self.running = True

        def poll(self):
            return None if self.running else 0

        def wait(self, timeout):
            self.running = False
            return 0

    process = Process()
    monkeypatch.setattr(
        cli.os, "killpg", lambda pid, value: signals.append((pid, value))
    )

    cli._stop_managed_launch(process)

    assert signals == [(12345, signal.SIGINT)]


def test_build_parser_manages_simulation_by_default() -> None:
    args = cli.build_parser().parse_args(
        ["build-recording", "--env", "lab_wall_a", "--trajectory", "circle_01"]
    )

    assert args.external_runtime is False
    assert args.launch_timeout == pytest.approx(120.0)


def test_play_parser_keeps_nucleo_capture_opt_in() -> None:
    args = cli.build_parser().parse_args(
        ["play", "--env", "lab_wall_a", "--recording", "circle_01__recording"]
    )

    assert args.record_nucleo is False
    assert args.nucleo_baudrate == 115200
    assert args.nucleo_channels == (1, 2, 3, 4)
    assert args.nucleo_ready_timeout == pytest.approx(5.0)
    assert args.record_tracker is False
    assert args.tracker_topic is None
    assert args.tracker_frame == "plane1"
    assert args.tracker_max_mapping_offset_ms == pytest.approx(50.0)


def test_play_parser_accepts_explicit_nucleo_capture_settings() -> None:
    args = cli.build_parser().parse_args(
        [
            "play",
            "--env",
            "lab_wall_a",
            "--recording",
            "circle_01__recording",
            "--record-nucleo",
            "--nucleo-port",
            "/dev/serial/by-id/nucleo",
            "--nucleo-baudrate",
            "230400",
            "--nucleo-channels",
            "1,3",
            "--nucleo-ready-timeout",
            "8",
        ]
    )

    assert args.record_nucleo is True
    assert args.nucleo_port == "/dev/serial/by-id/nucleo"
    assert args.nucleo_baudrate == 230400
    assert args.nucleo_channels == (1, 3)
    assert args.nucleo_ready_timeout == pytest.approx(8.0)


def test_nucleo_channel_parser_rejects_duplicates_and_unknown_channels() -> None:
    with pytest.raises(SystemExit):
        cli.build_parser().parse_args(
            [
                "play",
                "--env",
                "lab_wall_a",
                "--recording",
                "circle_01__recording",
                "--nucleo-channels",
                "1,1",
            ]
        )
    with pytest.raises(SystemExit):
        cli.build_parser().parse_args(
            [
                "play",
                "--env",
                "lab_wall_a",
                "--recording",
                "circle_01__recording",
                "--nucleo-channels",
                "5",
            ]
        )


def test_nucleo_preflight_failure_uses_tipdraw_error_path() -> None:
    capture_session = SimpleNamespace(
        start=lambda: (_ for _ in ()).throw(RuntimeError("SYNC ACK timed out"))
    )

    with pytest.raises(WaypointError, match="NUCLEO preflight failed.*SYNC ACK"):
        cli._arm_nucleo_capture(capture_session)


def test_robot_failure_without_telemetry_path_remains_authoritative() -> None:
    capture_summary = SimpleNamespace(staging_path="capture.staging.csv")
    failed_result = SimpleNamespace(
        success=False,
        telemetry_path="",
        message="recording validation failed",
    )

    assert cli._nucleo_robot_telemetry_path(capture_summary, failed_result) is None

    successful_result = SimpleNamespace(success=True, telemetry_path="")
    with pytest.raises(WaypointError, match="no robot telemetry path"):
        cli._nucleo_robot_telemetry_path(capture_summary, successful_result)

    assert cli._tracker_robot_telemetry_path(capture_summary, failed_result) is None
    failed_result.telemetry_path = "/tmp/partial.csv"
    assert cli._tracker_robot_telemetry_path(capture_summary, failed_result) is None
    with pytest.raises(WaypointError, match="no robot telemetry path"):
        cli._tracker_robot_telemetry_path(capture_summary, successful_result)


def test_play_parser_accepts_tracker_capture_settings() -> None:
    args = cli.build_parser().parse_args(
        [
            "play",
            "--env",
            "new_board_4cm_rod33365",
            "--recording",
            "yaw__recording",
            "--record-tracker",
            "--tracker-topic",
            "/tracker/pose",
            "--tracker-frame",
            "plane1",
            "--tracker-ready-timeout",
            "7",
            "--tracker-max-mapping-offset-ms",
            "20",
            "--tracker-minimum-samples",
            "200",
        ]
    )

    assert args.record_tracker is True
    assert args.tracker_topic == "/tracker/pose"
    assert args.tracker_frame == "plane1"
    assert args.tracker_ready_timeout == pytest.approx(7.0)
    assert args.tracker_max_mapping_offset_ms == pytest.approx(20.0)
    assert args.tracker_minimum_samples == 200


def test_yaw_generation_and_runner_parser_defaults() -> None:
    generated = cli.build_parser().parse_args(
        [
            "generate-yaw-calibration",
            "--env",
            "new_board_4cm_rod33365",
            "--trajectory-prefix",
            "yaw_calibration_center",
        ]
    )
    assert generated.sensor_distance_mm == pytest.approx(67.5)
    assert generated.sweep_deg == pytest.approx(360.0)
    assert generated.leg_duration_s == pytest.approx(18.0)
    assert generated.warmup_settle_s == pytest.approx(2.0)
    assert generated.handler is cli._generate_yaw_calibration

    runner = cli.build_parser().parse_args(
        [
            "yaw-calibrate",
            "--env",
            "new_board_4cm_rod33365",
            "--recording",
            "yaw__recording",
            "--tracker-topic",
            "/tracker/pose",
        ]
    )
    assert runner.tracker_frame == "plane1"
    assert runner.handler is cli._yaw_calibrate


def test_pendant_to_tf_parser_uses_base_plane1_defaults() -> None:
    args = cli.build_parser().parse_args(
        ["pendant-to-tf", "--pose", "p[1,2,3,0,0,0]"]
    )

    assert args.pose == "p[1,2,3,0,0,0]"
    assert args.parent_frame == "base"
    assert args.child_frame == "plane1"
    assert args.handler is cli._pendant_to_tf


def test_pendant_to_tf_prints_setup_value_and_copyable_command(capsys) -> None:
    args = cli.build_parser().parse_args(
        ["pendant-to-tf", "--pose", "p[1,2,3,0,0,0]"]
    )

    assert args.handler(args) == 0

    output = capsys.readouterr().out
    assert "translation_m: [1.0, 2.0, 3.0]" in output
    assert "quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]" in output
    assert "teach_pendant_pose_xyz_rotvec: [1.0, 2.0, 3.0, 0.0, 0.0, 0.0]" in output
    assert "--frame-id base --child-frame-id plane1" in output


def test_get_plane_parser_defaults_to_lab_ur5() -> None:
    args = cli.build_parser().parse_args(["get-plane"])

    assert args.robot_ip == "192.168.1.99"
    assert args.variable == "opt1"
    assert args.port == 29999
    assert args.timeout == pytest.approx(3.0)
    assert args.handler is cli._get_plane


def test_get_plane_prints_response_and_keeps_banner_on_stderr(
    monkeypatch, capsys
) -> None:
    calls: list[dict[str, object]] = []

    def fake_query(**kwargs):
        calls.append(kwargs)
        return SimpleNamespace(banner="Connected: UR", response="opt1 = p[0,0,0,0,0,0]")

    monkeypatch.setattr(cli, "query_dashboard_variable", fake_query)
    args = cli.build_parser().parse_args(
        [
            "get-plane",
            "--robot-ip",
            "192.168.1.99",
            "--variable",
            "opt1",
            "--port",
            "29999",
            "--timeout",
            "4",
        ]
    )

    assert args.handler(args) == 0

    captured = capsys.readouterr()
    assert captured.out == "opt1 = p[0,0,0,0,0,0]\n"
    assert captured.err == "UR Dashboard welcome: Connected: UR\n"
    assert calls == [
        {
            "robot_ip": "192.168.1.99",
            "variable": "opt1",
            "port": 29999,
            "timeout_s": 4.0,
        }
    ]


def test_get_plane_tf_parser_accepts_explicit_robot_and_plane_name() -> None:
    args = cli.build_parser().parse_args(
        [
            "get-plane-tf",
            "--robot-ip",
            "192.168.1.99",
            "--plane-name",
            "opt1",
        ]
    )

    assert args.robot_ip == "192.168.1.99"
    assert args.plane_name == "opt1"
    assert args.port == 29999
    assert args.timeout == pytest.approx(3.0)
    assert args.parent_frame == "base"
    assert args.child_frame == "plane1"
    assert args.handler is cli._get_plane_tf


def test_get_plane_tf_requires_explicit_robot_ip_and_plane_name() -> None:
    with pytest.raises(SystemExit):
        cli.build_parser().parse_args(["get-plane-tf", "--plane-name", "opt1"])
    with pytest.raises(SystemExit):
        cli.build_parser().parse_args(
            ["get-plane-tf", "--robot-ip", "192.168.1.99"]
        )


def test_get_plane_tf_queries_extracts_and_prints_copyable_command(
    monkeypatch, capsys
) -> None:
    calls: list[dict[str, object]] = []

    def fake_query(**kwargs):
        calls.append(kwargs)
        return SimpleNamespace(
            variable="opt1",
            banner="Connected: UR",
            response="VariableValue: p[1,2,3,0,0,0]",
        )

    monkeypatch.setattr(cli, "query_dashboard_variable", fake_query)
    args = cli.build_parser().parse_args(
        [
            "get-plane-tf",
            "--robot-ip",
            "192.168.1.99",
            "--plane-name",
            "opt1",
        ]
    )

    assert args.handler(args) == 0

    captured = capsys.readouterr()
    assert "translation_m: [1.0, 2.0, 3.0]" in captured.out
    assert "--frame-id base --child-frame-id plane1" in captured.out
    assert "UR Dashboard welcome: Connected: UR" in captured.err
    assert "VariableValue: p[1,2,3,0,0,0]" in captured.err
    assert calls == [
        {
            "robot_ip": "192.168.1.99",
            "variable": "opt1",
            "port": 29999,
            "timeout_s": 3.0,
        }
    ]
