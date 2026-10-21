"""Keep heavy runtime callbacks off the default parameter-service callback group."""

from pathlib import Path


def test_runtime_actions_share_exclusive_group_and_prepare_is_separate(
    repository_root: Path,
) -> None:
    source = (
        repository_root / "src/hx_waypoint_runtime/src/runtime_node.cpp"
    ).read_text(encoding="utf-8")
    assert (
        "action_callback_group_ = create_callback_group("
        "rclcpp::CallbackGroupType::MutuallyExclusive);"
    ) in source
    assert source.count(
        "rcl_action_server_get_default_options(), action_callback_group_)"
    ) == 2
    assert (
        "prepare_callback_group_ = create_callback_group("
        "rclcpp::CallbackGroupType::MutuallyExclusive);"
    ) in source
    assert (
        "prepare_plan_service_name_, rclcpp::ServicesQoS(), prepare_callback_group_)"
    ) in source
    # The latest-state and speed-scaling safety callbacks retain their existing concurrency.
    assert (
        "joint_state_callback_group_ = create_callback_group(\n"
        "      rclcpp::CallbackGroupType::Reentrant);"
    ) in source


def test_each_playback_client_and_both_watchdogs_share_one_retained_group(
    repository_root: Path,
) -> None:
    package = repository_root / "src/hx_waypoint_runtime"
    header = (package / "include/hx_waypoint/playback_executor.hpp").read_text(
        encoding="utf-8"
    )
    source = (package / "src/playback_executor.cpp").read_text(encoding="utf-8")
    assert "rclcpp::CallbackGroup::SharedPtr callback_group = nullptr" in header
    assert "rclcpp::CallbackGroup::SharedPtr callback_group_;" in header
    assert "callback_group ? std::move(callback_group) : node_->create_callback_group(" in source
    assert "rclcpp::CallbackGroupType::MutuallyExclusive" in source
    assert "node_, action_name_, callback_group_)" in source
    assert source.count("create_wall_timer(") == 2
    assert source.count("}, callback_group_);") == 2


def test_recorders_keep_subscription_order_without_blocking_parameter_services(
    repository_root: Path,
) -> None:
    package = repository_root / "src/hx_waypoint_runtime"
    telemetry = (package / "src/telemetry_recorder.cpp").read_text(encoding="utf-8")
    simulation = (package / "src/trajectory_recorder.cpp").read_text(encoding="utf-8")
    assert (
        "joint_callback_group_ = node_->create_callback_group(\n"
        "    rclcpp::CallbackGroupType::MutuallyExclusive);"
    ) in telemetry
    assert "joint_subscription_options.callback_group = joint_callback_group_;" in telemetry
    assert "std::placeholders::_1),\n    joint_subscription_options);" in telemetry
    assert (
        "speed_scaling_callback_group_ = node_->create_callback_group(\n"
        "      rclcpp::CallbackGroupType::Reentrant);"
    ) in telemetry
    assert (
        "callback_group_ = node_->create_callback_group("
        "rclcpp::CallbackGroupType::MutuallyExclusive);"
    ) in simulation
    assert "subscription_options.callback_group = callback_group_;" in simulation
    assert simulation.count("subscription_options);") == 2


def test_real_ros_responsiveness_regression_is_registered(repository_root: Path) -> None:
    package = repository_root / "src/hx_waypoint_runtime"
    cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")
    regression = (package / "test/test_callback_group_responsiveness.cpp").read_text(
        encoding="utf-8"
    )
    assert "ament_add_gtest(test_callback_group_responsiveness" in cmake
    assert "ParametersRespondDuringRepeatedBlockedControllerResults" in regression
    assert "rclcpp::AsyncParametersClient" in regression
    assert "parameters->get_parameters" in regression
    assert "cycle < 12U" in regression
    assert "goal->abort(result);" in regression
