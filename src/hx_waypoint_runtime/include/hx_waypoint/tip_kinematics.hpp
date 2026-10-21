#pragma once

#include <array>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace hx_waypoint
{

struct TipKinematicsResult
{
  std::array<double, 3> position_m{};
  std::array<double, 4> quaternion_xyzw{{0.0, 0.0, 0.0, 1.0}};
  std::array<double, 3> linear_velocity_m_s{};
  std::array<double, 3> angular_velocity_rad_s{};
};

struct FrameKinematicsResult
{
  std::string frame_id;
  std::array<double, 3> position_m{};
  std::array<double, 4> quaternion_xyzw{{0.0, 0.0, 0.0, 1.0}};
};

class TipKinematics
{
public:
  TipKinematics(
    const std::filesystem::path & urdf_path,
    std::string base_link,
    std::string tool_link,
    std::string tip_frame,
    const std::array<double, 3> & tool_to_tip_translation_m,
    const std::array<double, 4> & tool_to_tip_quaternion_xyzw,
    std::vector<std::string> joint_names,
    std::vector<std::string> frame_names = {});
  ~TipKinematics();

  TipKinematics(TipKinematics &&) noexcept;
  TipKinematics & operator=(TipKinematics &&) noexcept;
  TipKinematics(const TipKinematics &) = delete;
  TipKinematics & operator=(const TipKinematics &) = delete;

  [[nodiscard]] TipKinematicsResult evaluate(
    const std::vector<double> & positions,
    const std::vector<double> & velocities);

  [[nodiscard]] std::vector<FrameKinematicsResult> evaluateFrames(
    const std::vector<double> & positions);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace hx_waypoint
