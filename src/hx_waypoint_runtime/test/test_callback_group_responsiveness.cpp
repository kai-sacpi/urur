#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "hx_waypoint/playback_executor.hpp"

namespace
{

class ScopedContext
{
public:
  ScopedContext()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  ~ScopedContext() {rclcpp::shutdown();}
};

class ExecutorThread
{
public:
  explicit ExecutorThread(rclcpp::Executor & executor)
  : executor_(executor), thread_([this]() {executor_.spin();}) {}
  ~ExecutorThread()
  {
    executor_.cancel();
    thread_.join();
  }

private:
  rclcpp::Executor & executor_;
  std::thread thread_;
};

struct BlockingCallback
{
  void wait()
  {
    std::unique_lock<std::mutex> lock(mutex);
    entered = true;
    condition.notify_all();
    // Finite even on a failed assertion: no test can leave an executor thread hung.
    condition.wait_for(lock, std::chrono::seconds(5), [this]() {return released;});
  }

  bool waitUntilEntered()
  {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, std::chrono::seconds(2), [this]() {return entered;});
  }

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex);
    released = true;
    condition.notify_all();
  }

  std::mutex mutex;
  std::condition_variable condition;
  bool entered{false};
  bool released{false};
};

struct CycleResult
{
  std::promise<void> completed;
  std::atomic_int result_code{-1};
  std::atomic_bool failed{false};
};

trajectory_msgs::msg::JointTrajectory completeTrajectory()
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_a"};
  trajectory_msgs::msg::JointTrajectoryPoint start;
  start.positions = {0.0};
  start.velocities = {0.0};
  start.accelerations = {0.0};
  trajectory.points.push_back(start);
  auto finish = start;
  finish.positions = {0.1};
  finish.time_from_start.sec = 1;
  trajectory.points.push_back(finish);
  return trajectory;
}

// A controller result may do slow telemetry finalization. Exercise a real action and real
// GetParameters service, rather than treating graph discovery as responsiveness evidence.
TEST(RuntimeCallbackGroups, ParametersRespondDuringRepeatedBlockedControllerResults)
{
  ScopedContext context;
  using Follow = control_msgs::action::FollowJointTrajectory;
  using ServerGoal = rclcpp_action::ServerGoalHandle<Follow>;
  auto runtime = std::make_shared<rclcpp::Node>("callback_isolation_runtime");
  auto controller = std::make_shared<rclcpp::Node>("callback_isolation_controller");
  auto observer = std::make_shared<rclcpp::Node>("callback_isolation_observer");
  runtime->declare_parameter<std::int64_t>("responsiveness_probe", 73);
  const std::string action_name = "/callback_isolation/follow_joint_trajectory";
  std::atomic_uint accepted_count{0};
  auto server = rclcpp_action::create_server<Follow>(
    controller, action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Follow::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<ServerGoal>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [&accepted_count](const std::shared_ptr<ServerGoal> goal) {
      auto result = std::make_shared<Follow::Result>();
      if (accepted_count.fetch_add(1) % 2U == 0U) {
        result->error_code = Follow::Result::SUCCESSFUL;
        goal->succeed(result);
      } else {
        result->error_code = Follow::Result::PATH_TOLERANCE_VIOLATED;
        goal->abort(result);
      }
    });
  hx_waypoint::PlaybackExecutor playback(runtime.get(), action_name);
  auto parameters = std::make_shared<rclcpp::AsyncParametersClient>(observer, runtime->get_name());
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4U, false, std::chrono::milliseconds(50));
  executor.add_node(runtime);
  executor.add_node(controller);
  executor.add_node(observer);
  ExecutorThread spin(executor);
  ASSERT_TRUE(playback.waitForServer(std::chrono::seconds(2)));
  ASSERT_TRUE(parameters->wait_for_service(std::chrono::seconds(2)));

  // Include success and controller-abort cleanup, beyond the eleven-play incident prefix.
  for (unsigned int cycle = 0; cycle < 12U; ++cycle) {
    SCOPED_TRACE(cycle);
    auto block = std::make_shared<BlockingCallback>();
    auto result_state = std::make_shared<CycleResult>();
    auto completion = result_state->completed.get_future();
    hx_waypoint::PlaybackExecutor::Callbacks callbacks;
    callbacks.completed = [block, result_state](const auto & result) {
        result_state->result_code.store(static_cast<int>(result.code));
        block->wait();
        result_state->completed.set_value();
      };
    callbacks.failed_to_start = [result_state](const std::string &) {
        result_state->failed.store(true);
        result_state->completed.set_value();
      };
    const bool sent = playback.sendTrajectory(completeTrajectory(), std::move(callbacks));
    const bool callback_entered = sent && block->waitUntilEntered();
    EXPECT_TRUE(sent);
    EXPECT_TRUE(callback_entered);
    if (callback_entered) {
      for (unsigned int query = 0; query < 3U; ++query) {
        auto response = parameters->get_parameters({"responsiveness_probe"});
        const bool ready = response.wait_for(std::chrono::milliseconds(500)) ==
          std::future_status::ready;
        EXPECT_TRUE(ready) << "GetParameters blocked behind a controller result callback";
        if (ready) {
          const auto values = response.get();
          EXPECT_EQ(values.size(), 1U);
          if (values.size() == 1U) {
            EXPECT_EQ(values.front().as_int(), 73);
          }
        }
      }
    }
    block->release();
    const bool terminal = completion.wait_for(std::chrono::seconds(2)) ==
      std::future_status::ready;
    EXPECT_TRUE(terminal);
    EXPECT_FALSE(result_state->failed.load());
    EXPECT_EQ(result_state->result_code.load(), static_cast<int>(cycle % 2U == 0U ?
      rclcpp_action::ResultCode::SUCCEEDED : rclcpp_action::ResultCode::ABORTED));
    if (!terminal) {
      playback.shutdown();
      break;
    }
  }
  playback.shutdown();
  (void)server;
}

}  // namespace
