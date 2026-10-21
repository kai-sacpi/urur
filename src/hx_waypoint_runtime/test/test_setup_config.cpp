#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "hx_waypoint/setup_config.hpp"

namespace
{

constexpr double kHalfPi = 1.57079632679489661923;

class TemporarySetup
{
public:
  TemporarySetup()
  : root_(std::filesystem::temp_directory_path() /
      ("hx_waypoint_setup_config_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count())))
  {
    std::filesystem::create_directories(root_);
  }

  ~TemporarySetup()
  {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  TemporarySetup(const TemporarySetup &) = delete;
  TemporarySetup & operator=(const TemporarySetup &) = delete;

  [[nodiscard]] std::filesystem::path write(const std::string & work_frame_fields) const
  {
    const auto path = root_ / "setup.yaml";
    std::ofstream output(path);
    if (!output) {
      throw std::runtime_error("failed to create temporary setup.yaml");
    }
    output << std::setprecision(17) <<
      "schema_version: 1\n"
      "setup_id: test_setup\n"
      "work_frame:\n"
      "  parent_frame: base\n"
      "  child_frame: plane1\n"
      "  teach_pendant_pose_xyz_rotvec: [1.0, 2.0, 3.0, 0.0, 0.0, " <<
      kHalfPi << "]\n" <<
      work_frame_fields <<
      "  aliases: []\n"
      "tool:\n"
      "  tool_id: test_rod\n"
      "  parent_link: tool0\n"
      "  tip_frame: rod_tip\n"
      "  tip_orientation_xyzw: [0.0, 0.0, 0.0, 1.0]\n"
      "  rod:\n"
      "    axis_in_parent_xyz: [0.0, 0.0, 1.0]\n"
      "    length_m: 0.1\n"
      "    radius_m: 0.002\n"
      "    sphere_spacing_m: 0.003\n";
    if (!output) {
      throw std::runtime_error("failed to write temporary setup.yaml");
    }
    return path;
  }

private:
  std::filesystem::path root_;
};

void expectArrayNear(
  const std::array<double, 3> & actual,
  const std::array<double, 3> & expected,
  double tolerance = 1.0e-12)
{
  for (std::size_t index = 0U; index < actual.size(); ++index) {
    EXPECT_NEAR(actual[index], expected[index], tolerance) << "index=" << index;
  }
}

}  // namespace

TEST(SetupConfig, OptionalFeatureOffsetDefaultsToZeroAndPreservesRawPose)
{
  TemporarySetup files;
  const auto setup = hx_waypoint::loadSetupConfig(files.write(""));
  const auto raw = setup.work_frame.parentToFeature();
  const auto resolved = setup.work_frame.parentToChild();

  expectArrayNear(raw.translation_m, {{1.0, 2.0, 3.0}});
  expectArrayNear(resolved.translation_m, raw.translation_m);
  expectArrayNear(setup.work_frame.feature_to_work_frame_translation_m,
    {{0.0, 0.0, 0.0}});
  for (std::size_t index = 0U; index < raw.quaternion_xyzw.size(); ++index) {
    EXPECT_DOUBLE_EQ(raw.quaternion_xyzw[index], resolved.quaternion_xyzw[index]);
  }
}

TEST(SetupConfig, ComposesLocalFeatureOffsetUsingFeatureOrientation)
{
  TemporarySetup files;
  const auto setup = hx_waypoint::loadSetupConfig(
    files.write("  feature_to_work_frame_translation_m: [0.0, -0.025, 0.0]\n"));
  const auto raw = setup.work_frame.parentToFeature();
  const auto resolved = setup.work_frame.parentToChild();

  // The raw Feature is rotated +90 degrees around Z.  A local -Y 25 mm
  // offset therefore becomes a parent-frame +X 25 mm offset.
  expectArrayNear(raw.translation_m, {{1.0, 2.0, 3.0}});
  expectArrayNear(resolved.translation_m, {{1.025, 2.0, 3.0}});
  for (std::size_t index = 0U; index < raw.quaternion_xyzw.size(); ++index) {
    EXPECT_DOUBLE_EQ(raw.quaternion_xyzw[index], resolved.quaternion_xyzw[index]);
  }
  expectArrayNear(setup.work_frame.feature_to_work_frame_translation_m,
    {{0.0, -0.025, 0.0}});
}

TEST(SetupConfig, FeatureOffsetMustContainExactlyThreeFiniteNumbers)
{
  TemporarySetup files;
  EXPECT_THROW(
    hx_waypoint::loadSetupConfig(
      files.write("  feature_to_work_frame_translation_m: [0.0, -0.025]\n")),
    std::runtime_error);
  EXPECT_THROW(
    hx_waypoint::loadSetupConfig(
      files.write("  feature_to_work_frame_translation_m: [0.0, -0.025, nan]\n")),
    std::runtime_error);
  EXPECT_THROW(
    hx_waypoint::loadSetupConfig(
      files.write("  feature_to_work_frame_translation_m: null\n")),
    std::runtime_error);
}
