#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace hx_waypoint
{

class PlaybackExecutor
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  struct Callbacks
  {
    std::function<void()> accepted;
    std::function<void(const FollowJointTrajectory::Feedback &)> feedback;
    std::function<void(const GoalHandle::WrappedResult &)> completed;
    std::function<void(const std::string &)> failed_to_start;
  };

  PlaybackExecutor(
    rclcpp::Node * node,
    std::string action_name,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);
  ~PlaybackExecutor();

  PlaybackExecutor(const PlaybackExecutor &) = delete;
  PlaybackExecutor & operator=(const PlaybackExecutor &) = delete;
  PlaybackExecutor(PlaybackExecutor &&) = delete;
  PlaybackExecutor & operator=(PlaybackExecutor &&) = delete;

  [[nodiscard]] bool waitForServer(std::chrono::nanoseconds timeout) const;
  [[nodiscard]] bool sendTrajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    Callbacks callbacks);
  void cancelActive();
  void shutdown() noexcept;

  [[nodiscard]] bool inFlight() const;
  [[nodiscard]] const std::string & actionName() const noexcept;

private:
  struct SharedState;

  rclcpp::Node * node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
  std::string action_name_;
  std::shared_ptr<SharedState> state_;
};

}  // namespace hx_waypoint
