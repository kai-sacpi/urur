#include "hx_waypoint/playback_executor.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <rclcpp/timer.hpp>

namespace hx_waypoint
{
namespace
{

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr std::chrono::nanoseconds kGoalResponseTimeout = std::chrono::seconds(5);
constexpr std::chrono::nanoseconds kResultGrace = std::chrono::seconds(5);

[[nodiscard]] bool durationNanoseconds(
  const builtin_interfaces::msg::Duration & duration,
  std::int64_t & nanoseconds,
  std::string & error)
{
  if (duration.nanosec >= static_cast<std::uint32_t>(kNanosecondsPerSecond)) {
    error = "trajectory contains a non-canonical time_from_start nanosecond field";
    return false;
  }
  nanoseconds = static_cast<std::int64_t>(duration.sec) * kNanosecondsPerSecond +
    static_cast<std::int64_t>(duration.nanosec);
  return true;
}

[[nodiscard]] bool completeTrajectoryShape(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  std::chrono::nanoseconds & duration,
  std::string & error)
{
  if (trajectory.joint_names.empty() || trajectory.points.size() < 2U) {
    error = "trajectory must contain joint names and at least two points";
    return false;
  }

  std::int64_t previous_ns = -1;
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto & point = trajectory.points[index];
    if (point.positions.size() != trajectory.joint_names.size() ||
      point.velocities.size() != trajectory.joint_names.size() ||
      point.accelerations.size() != trajectory.joint_names.size())
    {
      error = "trajectory point " + std::to_string(index) +
        " does not contain complete q/qd/qdd vectors";
      return false;
    }
    for (const auto * values : {&point.positions, &point.velocities, &point.accelerations}) {
      for (const double value : *values) {
        if (!std::isfinite(value)) {
          error = "trajectory contains a non-finite command value";
          return false;
        }
      }
    }

    std::int64_t time_ns = 0;
    if (!durationNanoseconds(point.time_from_start, time_ns, error)) {
      return false;
    }
    if (index == 0U && time_ns != 0) {
      error = "trajectory first point must have time_from_start=0 exactly";
      return false;
    }
    if (index > 0U && time_ns <= previous_ns) {
      error = "trajectory time_from_start must be strictly increasing";
      return false;
    }
    previous_ns = time_ns;
  }

  if (previous_ns > std::chrono::nanoseconds::max().count() - kResultGrace.count()) {
    error = "trajectory duration exceeds the finite result-watchdog range";
    return false;
  }
  duration = std::chrono::nanoseconds(previous_ns);
  return true;
}

[[nodiscard]] std::string exceptionMessage(const char * prefix) noexcept
{
  try {
    throw;
  } catch (const std::exception & error) {
    return std::string(prefix) + error.what();
  } catch (...) {
    return std::string(prefix) + "unknown exception";
  }
}

}  // namespace

struct PlaybackExecutor::SharedState
{
  struct Request
  {
    Request(std::uint64_t request_generation, std::shared_ptr<Callbacks> request_callbacks)
    : generation(request_generation), callbacks(std::move(request_callbacks))
    {
    }

    const std::uint64_t generation;
    const std::shared_ptr<Callbacks> callbacks;
    std::atomic_bool cancel_requested{false};
    std::atomic_bool terminal{false};

    // The remaining fields are guarded by SharedState::mutex.
    bool request_in_flight{true};
    GoalHandle::SharedPtr active_goal;
    rclcpp::TimerBase::SharedPtr goal_response_watchdog;
    rclcpp::TimerBase::SharedPtr result_watchdog;
  };

  SharedState(
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client,
    std::string configured_action_name)
  : client(std::move(action_client)), action_name(std::move(configured_action_name))
  {
  }

  [[nodiscard]] bool isCurrentLocked(const std::shared_ptr<Request> & request) const noexcept
  {
    return current && current.get() == request.get() &&
           current->generation == request->generation;
  }

  static void cancelTimer(const rclcpp::TimerBase::SharedPtr & timer) noexcept
  {
    if (!timer) {
      return;
    }
    try {
      timer->cancel();
    } catch (...) {
      // A timer is only a local watchdog. Its generation/terminal checks still make a late
      // callback harmless if the underlying ROS timer cannot be canceled during shutdown.
    }
  }

  [[nodiscard]] static std::string requestGoalCancel(
    const rclcpp_action::Client<FollowJointTrajectory>::SharedPtr & action_client,
    const GoalHandle::SharedPtr & goal) noexcept
  {
    if (!action_client || !goal) {
      return {};
    }
    try {
      (void)action_client->async_cancel_goal(goal);
      return {};
    } catch (...) {
      return exceptionMessage("failed to cancel controller trajectory goal: ");
    }
  }

  [[nodiscard]] static std::string stopGoalCallbacks(
    const rclcpp_action::Client<FollowJointTrajectory>::SharedPtr & action_client,
    const GoalHandle::SharedPtr & goal) noexcept
  {
    if (!action_client || !goal) {
      return {};
    }
    try {
      action_client->stop_callbacks(goal);
      return {};
    } catch (...) {
      return exceptionMessage("failed to detach stale controller goal callbacks: ");
    }
  }

  void invokeImmediateFailure(const Callbacks & callbacks, const std::string & message) noexcept
  {
    std::lock_guard<std::recursive_mutex> callback_lock(callback_mutex);
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (shutting_down) {
        return;
      }
    }
    if (callbacks.failed_to_start) {
      try {
        callbacks.failed_to_start(message);
      } catch (...) {
        // User callbacks must not unwind through an rclcpp executor callback.
      }
    }
  }

  void invokeFailure(
    const std::shared_ptr<Request> & request,
    const std::string & message) noexcept
  {
    std::lock_guard<std::recursive_mutex> callback_lock(callback_mutex);
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (shutting_down) {
        return;
      }
    }
    if (request->callbacks->failed_to_start) {
      try {
        request->callbacks->failed_to_start(message);
      } catch (...) {
        // The request is already terminal and detached; suppress callback exceptions.
      }
    }
  }

  [[nodiscard]] std::string invokeAccepted(const std::shared_ptr<Request> & request) noexcept
  {
    std::lock_guard<std::recursive_mutex> callback_lock(callback_mutex);
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (shutting_down || !isCurrentLocked(request) || request->terminal.load()) {
        return {};
      }
    }
    if (!request->callbacks->accepted) {
      return {};
    }
    try {
      request->callbacks->accepted();
      return {};
    } catch (...) {
      return exceptionMessage("trajectory accepted callback raised an exception: ");
    }
  }

  [[nodiscard]] std::string invokeFeedback(
    const std::shared_ptr<Request> & request,
    const FollowJointTrajectory::Feedback & feedback) noexcept
  {
    std::lock_guard<std::recursive_mutex> callback_lock(callback_mutex);
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (shutting_down || !isCurrentLocked(request) || request->terminal.load()) {
        return {};
      }
    }
    if (!request->callbacks->feedback) {
      return {};
    }
    try {
      request->callbacks->feedback(feedback);
      return {};
    } catch (...) {
      return exceptionMessage("trajectory feedback callback raised an exception: ");
    }
  }

  void invokeCompleted(
    const std::shared_ptr<Request> & request,
    const GoalHandle::WrappedResult & result) noexcept
  {
    std::lock_guard<std::recursive_mutex> callback_lock(callback_mutex);
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (shutting_down) {
        return;
      }
    }
    if (request->callbacks->completed) {
      try {
        request->callbacks->completed(result);
      } catch (...) {
        // The controller result is already terminal; no second terminal callback is emitted.
      }
    }
  }

  void fail(
    const std::shared_ptr<Request> & request,
    std::string message,
    bool cancel_active_goal) noexcept
  {
    // Serialize the terminal transition with shutdown(). This prevents shutdown from slipping
    // between detaching the request and delivering its terminal callback.
    std::lock_guard<std::recursive_mutex> callback_lock(callback_mutex);
    GoalHandle::SharedPtr goal;
    rclcpp::TimerBase::SharedPtr response_watchdog;
    rclcpp::TimerBase::SharedPtr result_watchdog;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!isCurrentLocked(request) || request->terminal.exchange(true)) {
        return;
      }
      request->request_in_flight = false;
      goal = std::move(request->active_goal);
      response_watchdog = request->goal_response_watchdog;
      result_watchdog = request->result_watchdog;
      current.reset();
    }

    cancelTimer(response_watchdog);
    cancelTimer(result_watchdog);
    if (cancel_active_goal && goal) {
      const std::string cancel_error = requestGoalCancel(client, goal);
      const std::string stop_error = stopGoalCallbacks(client, goal);
      if (!cancel_error.empty()) {
        message += "; " + cancel_error;
      }
      if (!stop_error.empty()) {
        message += "; " + stop_error;
      }
    }
    invokeFailure(request, message);
  }

  void goalResponse(
    const std::shared_ptr<Request> & request,
    const GoalHandle::SharedPtr & goal_handle) noexcept
  {
    if (!goal_handle) {
      fail(
        request,
        "controller rejected the complete trajectory goal on " + action_name,
        false);
      return;
    }

    bool stale = false;
    bool cancel_latched = false;
    std::string watchdog_error;
    {
      std::lock_guard<std::mutex> lock(mutex);
      stale = shutting_down || !isCurrentLocked(request) || request->terminal.load();
      if (!stale) {
        request->request_in_flight = false;
        request->active_goal = goal_handle;
        cancelTimer(request->goal_response_watchdog);
        if (!request->result_watchdog) {
          watchdog_error = "result watchdog was not initialized";
        } else {
          try {
            request->result_watchdog->reset();
          } catch (...) {
            watchdog_error = exceptionMessage("failed to arm result watchdog: ");
          }
        }
        cancel_latched = request->cancel_requested.load();
      }
    }

    if (stale) {
      // A timed-out, canceled-and-shut-down, or superseded request must never be resurrected by
      // a late goal response. If the controller accepted it, cancel that exact goal immediately.
      request->terminal.store(true);
      (void)requestGoalCancel(client, goal_handle);
      (void)stopGoalCallbacks(client, goal_handle);
      return;
    }
    if (!watchdog_error.empty()) {
      fail(request, watchdog_error, true);
      return;
    }
    if (cancel_latched) {
      const std::string cancel_error = requestGoalCancel(client, goal_handle);
      if (!cancel_error.empty()) {
        fail(request, cancel_error, true);
        return;
      }
    }

    const std::string callback_error = invokeAccepted(request);
    if (!callback_error.empty()) {
      fail(request, callback_error, true);
    }
  }

  void feedback(
    const std::shared_ptr<Request> & request,
    const FollowJointTrajectory::Feedback & message) noexcept
  {
    const std::string callback_error = invokeFeedback(request, message);
    if (!callback_error.empty()) {
      fail(request, callback_error, true);
    }
  }

  void result(
    const std::shared_ptr<Request> & request,
    const GoalHandle::WrappedResult & wrapped_result) noexcept
  {
    // Keep the state transition and terminal callback on the same side of the shutdown barrier.
    std::lock_guard<std::recursive_mutex> callback_lock(callback_mutex);
    rclcpp::TimerBase::SharedPtr response_watchdog;
    rclcpp::TimerBase::SharedPtr result_watchdog;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!isCurrentLocked(request) || request->terminal.exchange(true)) {
        return;
      }
      request->request_in_flight = false;
      request->active_goal.reset();
      response_watchdog = request->goal_response_watchdog;
      result_watchdog = request->result_watchdog;
      current.reset();
    }
    cancelTimer(response_watchdog);
    cancelTimer(result_watchdog);
    invokeCompleted(request, wrapped_result);
  }

  void cancelActive() noexcept
  {
    std::shared_ptr<Request> request;
    GoalHandle::SharedPtr goal;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (shutting_down || !current || current->terminal.load()) {
        return;
      }
      request = current;
      if (request->cancel_requested.exchange(true)) {
        return;
      }
      goal = request->active_goal;
    }

    // If the goal request is still pending, the latch above is sufficient. goalResponse() will
    // cancel the exact goal before invoking the accepted callback if a late acceptance arrives.
    if (goal) {
      const std::string error = requestGoalCancel(client, goal);
      if (!error.empty()) {
        fail(request, error, true);
      }
    }
  }

  void shutdown() noexcept
  {
    std::shared_ptr<Request> request;
    GoalHandle::SharedPtr goal;
    rclcpp::TimerBase::SharedPtr response_watchdog;
    rclcpp::TimerBase::SharedPtr result_watchdog;

    // callback_mutex is the destruction barrier for user callbacks. Holding it while setting the
    // terminal latch guarantees shutdown() returns only after any callback already in progress has
    // left, and no later callback can enter user code.
    std::lock_guard<std::recursive_mutex> callback_lock(callback_mutex);
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (shutting_down) {
        return;
      }
      shutting_down = true;
      request = std::move(current);
      if (request) {
        request->cancel_requested.store(true);
        request->terminal.store(true);
        request->request_in_flight = false;
        goal = std::move(request->active_goal);
        response_watchdog = request->goal_response_watchdog;
        result_watchdog = request->result_watchdog;
      }
    }

    cancelTimer(response_watchdog);
    cancelTimer(result_watchdog);
    if (goal) {
      (void)requestGoalCancel(client, goal);
      (void)stopGoalCallbacks(client, goal);
    }
    if (request && request->callbacks->failed_to_start) {
      try {
        request->callbacks->failed_to_start(
          "trajectory executor shut down before the controller operation reached a terminal "
          "result");
      } catch (...) {
        // shutdown() is noexcept and the request has already been made terminal and detached.
      }
    }
  }

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client;
  const std::string action_name;
  mutable std::mutex mutex;
  std::recursive_mutex callback_mutex;
  std::uint64_t next_generation{0U};
  bool shutting_down{false};
  std::shared_ptr<Request> current;
};

PlaybackExecutor::PlaybackExecutor(
  rclcpp::Node * node,
  std::string action_name,
  rclcpp::CallbackGroup::SharedPtr callback_group)
: node_(node), action_name_(std::move(action_name))
{
  if (node_ == nullptr) {
    throw std::invalid_argument("PlaybackExecutor requires a node");
  }
  if (action_name_.empty()) {
    throw std::invalid_argument("FollowJointTrajectory action_name must not be empty");
  }
  // Keep controller callbacks and their watchdogs serialized with each other, but never with
  // parameter services. A slow result callback must not hide an otherwise-live runtime node.
  callback_group_ = callback_group ? std::move(callback_group) : node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    node_, action_name_, callback_group_);
  state_ = std::make_shared<SharedState>(client_, action_name_);
}

PlaybackExecutor::~PlaybackExecutor()
{
  shutdown();
  state_.reset();
  client_.reset();
  node_ = nullptr;
}

bool PlaybackExecutor::waitForServer(std::chrono::nanoseconds timeout) const
{
  const auto state = state_;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (state->shutting_down) {
      return false;
    }
  }
  return client_->wait_for_action_server(timeout);
}

bool PlaybackExecutor::sendTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  Callbacks callbacks)
{
  const auto state = state_;
  std::chrono::nanoseconds trajectory_duration{0};
  std::string validation_error;
  if (!completeTrajectoryShape(trajectory, trajectory_duration, validation_error)) {
    state->invokeImmediateFailure(callbacks, validation_error);
    return false;
  }

  auto shared_callbacks = std::make_shared<Callbacks>(std::move(callbacks));
  std::shared_ptr<SharedState::Request> request;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (state->shutting_down) {
      validation_error = "trajectory executor has been shut down";
    } else if (state->current) {
      validation_error = "a trajectory goal is already active";
    } else if (state->next_generation == std::numeric_limits<std::uint64_t>::max()) {
      validation_error = "trajectory request generation counter is exhausted";
    } else {
      request = std::make_shared<SharedState::Request>(
        ++state->next_generation, shared_callbacks);
      state->current = request;
    }
  }
  if (!validation_error.empty()) {
    state->invokeImmediateFailure(*shared_callbacks, validation_error);
    return false;
  }

  const std::weak_ptr<SharedState> weak_state(state);
  const std::weak_ptr<SharedState::Request> weak_request(request);
  rclcpp::TimerBase::SharedPtr response_watchdog;
  rclcpp::TimerBase::SharedPtr result_watchdog;
  try {
    response_watchdog = node_->create_wall_timer(
      kGoalResponseTimeout,
      [weak_state, weak_request]() {
        const auto callback_state = weak_state.lock();
        const auto callback_request = weak_request.lock();
        if (callback_state && callback_request) {
          callback_state->fail(
            callback_request,
            "controller did not answer the complete trajectory goal within the finite "
            "5 second goal-response deadline",
            false);
        }
      }, callback_group_);
    result_watchdog = node_->create_wall_timer(
      trajectory_duration + kResultGrace,
      [weak_state, weak_request]() {
        const auto callback_state = weak_state.lock();
        const auto callback_request = weak_request.lock();
        if (callback_state && callback_request) {
          callback_state->fail(
            callback_request,
            "controller did not return a result within authoritative trajectory duration plus "
            "the finite 5 second grace period",
            true);
        }
      }, callback_group_);
    result_watchdog->cancel();
  } catch (...) {
    SharedState::cancelTimer(response_watchdog);
    SharedState::cancelTimer(result_watchdog);
    state->fail(
      request,
      exceptionMessage("failed to create trajectory lifecycle watchdogs: "),
      false);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (state->shutting_down || !state->isCurrentLocked(request) || request->terminal.load()) {
      SharedState::cancelTimer(response_watchdog);
      SharedState::cancelTimer(result_watchdog);
      return false;
    }
    request->goal_response_watchdog = response_watchdog;
    request->result_watchdog = result_watchdog;
  }

  FollowJointTrajectory::Goal goal;
  // This assignment is intentional: the complete authoritative q/qd/qdd/time trajectory is sent
  // once, in one FollowJointTrajectory goal. Playback never splits, scales, retimes, crops, or
  // skips any point.
  goal.trajectory = trajectory;

  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
  // Keep the client alive until the goal response arrives. This is required for a timed-out or
  // shutting-down request: a late acceptance still has to cancel the exact server-side goal.
  const auto response_client = client_;
  options.goal_response_callback =
    [weak_state, request, response_client](const GoalHandle::SharedPtr & goal_handle) {
      if (const auto callback_state = weak_state.lock()) {
        callback_state->goalResponse(request, goal_handle);
        return;
      }
      if (goal_handle) {
        request->terminal.store(true);
        request->cancel_requested.store(true);
        (void)SharedState::requestGoalCancel(response_client, goal_handle);
        (void)SharedState::stopGoalCallbacks(response_client, goal_handle);
      }
    };
  options.feedback_callback =
    [weak_state, weak_request](
      GoalHandle::SharedPtr,
      const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
      const auto callback_state = weak_state.lock();
      const auto callback_request = weak_request.lock();
      if (callback_state && callback_request && feedback) {
        callback_state->feedback(callback_request, *feedback);
      }
    };
  options.result_callback =
    [weak_state, weak_request](const GoalHandle::WrappedResult & result) {
      const auto callback_state = weak_state.lock();
      const auto callback_request = weak_request.lock();
      if (callback_state && callback_request) {
        callback_state->result(callback_request, result);
      }
    };

  try {
    (void)client_->async_send_goal(goal, options);
  } catch (...) {
    state->fail(
      request,
      exceptionMessage("failed to send complete trajectory goal: "),
      false);
    return false;
  }
  return true;
}

void PlaybackExecutor::cancelActive()
{
  state_->cancelActive();
}

void PlaybackExecutor::shutdown() noexcept
{
  if (state_) {
    state_->shutdown();
  }
}

bool PlaybackExecutor::inFlight() const
{
  const auto state = state_;
  std::lock_guard<std::mutex> lock(state->mutex);
  return !state->shutting_down && state->current && !state->current->terminal.load() &&
         (state->current->request_in_flight || state->current->active_goal);
}

const std::string & PlaybackExecutor::actionName() const noexcept
{
  return action_name_;
}

}  // namespace hx_waypoint
