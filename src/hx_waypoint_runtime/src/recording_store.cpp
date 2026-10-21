#include "hx_waypoint/recording_store.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <builtin_interfaces/msg/duration.hpp>
#include <yaml-cpp/yaml.h>

#include "hx_waypoint/canonical_trajectory_validator.hpp"
#include "hx_waypoint/robot_validation_model.hpp"
#include "hx_waypoint/runtime_policy.hpp"
#include "hx_waypoint/simulation_recording_validator.hpp"

namespace hx_waypoint
{
namespace
{

constexpr int kSupportedSchemaVersion = 1;
constexpr int kCompileReportSchemaVersion = 2;
constexpr double kTimeTolerance = 1.0e-6;
constexpr double kCanonicalDerivativeTolerance = 1.0e-9;
constexpr double kCuroboFeasibilityCostTolerance = 1.0e-7;
constexpr const char * kCompilerContractRevision = "fixed_time_waypoint_v2";
constexpr const char * kJointWindingPolicy =
  "constant_whole_path_nearest_home_without_margin_loss";
constexpr int kActiveLifecycleStateId = 3;
constexpr std::array<const char *, 10> kRequiredRecordingFiles{{
    "manifest.yaml", "source_tip_trajectory.csv", "joint_trajectory.csv", "joint_frames.csv",
    "sim_controller_state.csv", "sim_actual_joint.csv", "sim_actual_tip.csv",
    "compile_report.yaml", "validation_report.yaml", "sha256sums.txt"}};
constexpr std::array<const char *, 12> kRequiredEnvironmentFiles{{
    "environment.yaml", "setup.yaml", "zones.yaml", "planner.yaml",
    "recording.yaml", "controllers.yaml", "robot/robot_model.yaml",
    "robot/calibrated_kinematics.yaml", "robot/asset_provenance.yaml",
    "robot/driver_kinematics.yaml", "robot/curobo_robot.yaml",
    "robot/collision_spheres.yaml"}};
constexpr std::array<const char *, 2> kEnvironmentTreeDirectories{{
    "meshes", "robot/assets"}};

class Sha256
{
public:
  Sha256()
  : state_{
      0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
      0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U}
  {
  }

  void update(const std::uint8_t * data, std::size_t length)
  {
    for (std::size_t index = 0; index < length; ++index) {
      buffer_[buffer_size_++] = data[index];
      if (buffer_size_ == buffer_.size()) {
        transform(buffer_.data());
        bit_count_ += 512U;
        buffer_size_ = 0U;
      }
    }
  }

  [[nodiscard]] std::array<std::uint8_t, 32> finish()
  {
    const std::uint64_t total_bits = bit_count_ + static_cast<std::uint64_t>(buffer_size_) * 8U;
    buffer_[buffer_size_++] = 0x80U;

    if (buffer_size_ > 56U) {
      while (buffer_size_ < 64U) {
        buffer_[buffer_size_++] = 0U;
      }
      transform(buffer_.data());
      buffer_size_ = 0U;
    }
    while (buffer_size_ < 56U) {
      buffer_[buffer_size_++] = 0U;
    }
    for (int shift = 56; shift >= 0; shift -= 8) {
      buffer_[buffer_size_++] = static_cast<std::uint8_t>((total_bits >> shift) & 0xffU);
    }
    transform(buffer_.data());

    std::array<std::uint8_t, 32> digest{};
    for (std::size_t index = 0; index < state_.size(); ++index) {
      digest[index * 4U] = static_cast<std::uint8_t>((state_[index] >> 24U) & 0xffU);
      digest[index * 4U + 1U] = static_cast<std::uint8_t>((state_[index] >> 16U) & 0xffU);
      digest[index * 4U + 2U] = static_cast<std::uint8_t>((state_[index] >> 8U) & 0xffU);
      digest[index * 4U + 3U] = static_cast<std::uint8_t>(state_[index] & 0xffU);
    }
    return digest;
  }

private:
  static constexpr std::array<std::uint32_t, 64> kRoundConstants{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
    0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
    0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
    0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
    0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
    0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
    0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
    0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
    0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
    0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
    0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
    0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

  static std::uint32_t rotateRight(std::uint32_t value, std::uint32_t bits)
  {
    return (value >> bits) | (value << (32U - bits));
  }

  void transform(const std::uint8_t * block)
  {
    std::array<std::uint32_t, 64> words{};
    for (std::size_t index = 0; index < 16U; ++index) {
      const std::size_t offset = index * 4U;
      words[index] =
        (static_cast<std::uint32_t>(block[offset]) << 24U) |
        (static_cast<std::uint32_t>(block[offset + 1U]) << 16U) |
        (static_cast<std::uint32_t>(block[offset + 2U]) << 8U) |
        static_cast<std::uint32_t>(block[offset + 3U]);
    }
    for (std::size_t index = 16U; index < words.size(); ++index) {
      const std::uint32_t s0 =
        rotateRight(words[index - 15U], 7U) ^ rotateRight(words[index - 15U], 18U) ^
        (words[index - 15U] >> 3U);
      const std::uint32_t s1 =
        rotateRight(words[index - 2U], 17U) ^ rotateRight(words[index - 2U], 19U) ^
        (words[index - 2U] >> 10U);
      words[index] = words[index - 16U] + s0 + words[index - 7U] + s1;
    }

    std::uint32_t a = state_[0];
    std::uint32_t b = state_[1];
    std::uint32_t c = state_[2];
    std::uint32_t d = state_[3];
    std::uint32_t e = state_[4];
    std::uint32_t f = state_[5];
    std::uint32_t g = state_[6];
    std::uint32_t h = state_[7];

    for (std::size_t index = 0; index < words.size(); ++index) {
      const std::uint32_t sum1 =
        rotateRight(e, 6U) ^ rotateRight(e, 11U) ^ rotateRight(e, 25U);
      const std::uint32_t choose = (e & f) ^ ((~e) & g);
      const std::uint32_t temp1 = h + sum1 + choose + kRoundConstants[index] + words[index];
      const std::uint32_t sum0 =
        rotateRight(a, 2U) ^ rotateRight(a, 13U) ^ rotateRight(a, 22U);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temp2 = sum0 + majority;

      h = g;
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
    }

    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
  }

  std::array<std::uint32_t, 8> state_;
  std::array<std::uint8_t, 64> buffer_{};
  std::size_t buffer_size_{0U};
  std::uint64_t bit_count_{0U};
};

[[nodiscard]] std::string sha256Bytes(const std::string & bytes)
{
  Sha256 hash;
  if (!bytes.empty()) {
    hash.update(
      reinterpret_cast<const std::uint8_t *>(bytes.data()), bytes.size());
  }
  const auto digest = hash.finish();
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const std::uint8_t byte : digest) {
    output << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return output.str();
}

[[nodiscard]] std::string trim(std::string value)
{
  const auto not_space = [](unsigned char character) {return !std::isspace(character);};
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

[[nodiscard]] std::string normalizedHash(std::string value)
{
  value = trim(std::move(value));
  constexpr const char * prefix = "sha256:";
  if (value.rfind(prefix, 0U) == 0U) {
    value.erase(0U, std::strlen(prefix));
  }
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  if (value.size() != 64U || !std::all_of(value.begin(), value.end(), [](unsigned char character) {
      return std::isxdigit(character) != 0;
    }))
  {
    throw std::runtime_error("invalid SHA-256 value: " + value);
  }
  return value;
}

[[nodiscard]] std::string normalizedGitCommit(std::string value)
{
  value = trim(std::move(value));
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  if (value.size() != 40U || !std::all_of(value.begin(), value.end(), [](unsigned char character) {
      return std::isxdigit(character) != 0;
    }))
  {
    throw std::runtime_error("cuRobo revision must be a full 40-character Git commit: " + value);
  }
  return value;
}

[[nodiscard]] std::vector<double> requiredFiniteSequence(
  const YAML::Node & parent, const std::string & key, std::size_t expected_size)
{
  const YAML::Node values = parent[key];
  if (!values || !values.IsSequence() || values.size() != expected_size) {
    throw std::runtime_error(
            "compile_report.yaml " + key + " must contain exactly " +
            std::to_string(expected_size) + " values");
  }
  std::vector<double> result;
  result.reserve(expected_size);
  for (const auto & value : values) {
    const double parsed = value.as<double>();
    if (!std::isfinite(parsed)) {
      throw std::runtime_error("compile_report.yaml " + key + " contains a non-finite value");
    }
    result.push_back(parsed);
  }
  return result;
}

void requireRegularFile(const std::filesystem::path & path, const std::string & description)
{
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error(description + " does not exist or is not a regular file: " + path.string());
  }
}

void requireNonEmptyArtifactFile(
  const std::filesystem::path & path, const std::string & description)
{
  const auto status = std::filesystem::symlink_status(path);
  if (std::filesystem::is_symlink(status) || !std::filesystem::is_regular_file(status)) {
    throw std::runtime_error(
            description + " does not exist, is not a regular file, or is a symlink: " +
            path.string());
  }
  if (std::filesystem::file_size(path) == 0U) {
    throw std::runtime_error(description + " is empty: " + path.string());
  }
}

struct HashedYamlDocument
{
  YAML::Node root;
  std::string sha256;
};

struct HashedFileContents
{
  std::string bytes;
  std::string sha256;
};

[[nodiscard]] HashedFileContents loadHashedFileContents(
  const std::filesystem::path & path, const std::string & description)
{
  requireNonEmptyArtifactFile(path, description);
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to open " + description + ": " + path.string());
  }
  std::string bytes{
    std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  if (input.bad()) {
    throw std::runtime_error("failed while reading " + description + ": " + path.string());
  }
  const std::string digest = sha256Bytes(bytes);
  return HashedFileContents{std::move(bytes), digest};
}

[[nodiscard]] HashedYamlDocument loadHashedYamlDocument(
  const std::filesystem::path & path, const std::string & description)
{
  auto contents = loadHashedFileContents(path, description);
  try {
    return HashedYamlDocument{YAML::Load(contents.bytes), std::move(contents.sha256)};
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "failed to parse " + description + ": " + std::string(error.what()));
  }
}

[[nodiscard]] std::string requiredString(const YAML::Node & parent, const std::string & key)
{
  const YAML::Node value = parent[key];
  if (!value || !value.IsScalar()) {
    throw std::runtime_error("manifest field '" + key + "' must be a scalar");
  }
  const std::string result = trim(value.as<std::string>());
  if (result.empty()) {
    throw std::runtime_error("manifest field '" + key + "' must not be empty");
  }
  return result;
}

[[nodiscard]] std::vector<std::string> requiredStringSequence(
  const YAML::Node & parent, const std::string & key)
{
  const YAML::Node sequence = parent[key];
  if (!sequence || !sequence.IsSequence() || sequence.size() == 0U) {
    throw std::runtime_error("manifest field '" + key + "' must be a non-empty sequence");
  }
  std::vector<std::string> result;
  result.reserve(sequence.size());
  for (const auto & item : sequence) {
    if (!item.IsScalar()) {
      throw std::runtime_error("manifest field '" + key + "' contains a non-scalar item");
    }
    result.push_back(trim(item.as<std::string>()));
    if (result.back().empty()) {
      throw std::runtime_error("manifest field '" + key + "' contains an empty item");
    }
  }
  return result;
}

void parseAndValidateJointWindingEvidence(
  const YAML::Node & report,
  const YAML::Node & robot_model,
  const std::vector<std::string> & expected_joint_names,
  CompiledRunEvidence & evidence)
{
  const YAML::Node robot = robot_model["robot"];
  const YAML::Node robot_joint_names = robot["joint_names"];
  const YAML::Node home_positions = robot["home_joint_positions"];
  if (!robot || !robot.IsMap() || !robot_joint_names || !robot_joint_names.IsSequence() ||
    robot_joint_names.size() == 0U || !home_positions || !home_positions.IsSequence() ||
    home_positions.size() != robot_joint_names.size())
  {
    throw std::runtime_error("robot_model.yaml joint_names/home_joint_positions are invalid");
  }
  const auto configured_joint_names = robot_joint_names.as<std::vector<std::string>>();
  if (std::any_of(
      configured_joint_names.begin(), configured_joint_names.end(),
      [](const std::string & name) {return trim(name).empty();}) ||
    std::unordered_set<std::string>(
      configured_joint_names.begin(), configured_joint_names.end()).size() !=
    configured_joint_names.size())
  {
    throw std::runtime_error("robot_model.yaml joint_names must be unique and non-empty");
  }
  if (!expected_joint_names.empty() && configured_joint_names != expected_joint_names) {
    throw std::runtime_error(
            "recording environment_snapshot robot joint_names do not match the manifest");
  }

  const std::size_t joint_count = configured_joint_names.size();
  std::vector<double> configured_home;
  configured_home.reserve(joint_count);
  for (const auto & value : home_positions) {
    const double parsed = value.as<double>();
    if (!std::isfinite(parsed)) {
      throw std::runtime_error("robot_model.yaml home_joint_positions contains a non-finite value");
    }
    configured_home.push_back(parsed);
  }

  const YAML::Node winding = report["joint_winding"];
  if (!winding || !winding.IsMap()) {
    throw std::runtime_error("compile_report.yaml joint_winding must be a map");
  }
  evidence.joint_winding_policy = requiredString(winding, "policy");
  if (evidence.joint_winding_policy != kJointWindingPolicy) {
    throw std::runtime_error("compile_report.yaml joint_winding policy is unsupported");
  }
  evidence.joint_winding_reference_positions_rad = requiredFiniteSequence(
    winding, "reference_positions_rad", joint_count);
  evidence.selected_seed_offsets_rad = requiredFiniteSequence(
    winding, "selected_seed_offsets_rad", joint_count);
  evidence.post_optimization_offsets_rad = requiredFiniteSequence(
    winding, "post_optimization_offsets_rad", joint_count);
  for (std::size_t index = 0U; index < joint_count; ++index) {
    if (std::abs(
        evidence.joint_winding_reference_positions_rad[index] - configured_home[index]) >
      1.0e-12)
    {
      throw std::runtime_error(
              "compile_report.yaml joint_winding reference does not match robot home");
    }
    for (const double offset : {
        evidence.selected_seed_offsets_rad[index],
        evidence.post_optimization_offsets_rad[index]})
    {
      const double turns = offset / (2.0 * std::acos(-1.0));
      if (std::abs(turns - std::round(turns)) > 1.0e-9) {
        throw std::runtime_error(
                "compile_report.yaml joint_winding offset is not an integer full turn");
      }
    }
  }
}

[[nodiscard]] std::unordered_map<std::string, std::string> parseCompileEnvironmentHashes(
  const YAML::Node & report)
{
  const YAML::Node reported_hashes = report["environment_hashes"];
  if (!reported_hashes || !reported_hashes.IsMap() || reported_hashes.size() == 0U) {
    throw std::runtime_error("compile_report.yaml environment_hashes must be a non-empty map");
  }
  std::unordered_map<std::string, std::string> hashes;
  for (const auto & entry : reported_hashes) {
    if (!entry.first.IsScalar() || !entry.second.IsScalar()) {
      throw std::runtime_error(
              "compile_report.yaml environment_hashes entries must be scalar key/value pairs");
    }
    const std::string key = trim(entry.first.as<std::string>());
    const auto relative = std::filesystem::path(key).lexically_normal();
    const bool has_parent_reference = std::any_of(
      relative.begin(), relative.end(), [](const auto & component) {
        return component == "..";
      });
    if (key.empty() || relative.is_absolute() || relative == "." || has_parent_reference ||
      key.find('\\') != std::string::npos || relative.generic_string() != key)
    {
      throw std::runtime_error("unsafe path in compile_report.yaml environment_hashes: " + key);
    }
    if (!hashes.emplace(key, normalizedHash(entry.second.as<std::string>())).second) {
      throw std::runtime_error(
              "duplicate path in compile_report.yaml environment_hashes: " + key);
    }
  }
  return hashes;
}

[[nodiscard]] CompiledRunEvidence parseCompileReportEvidence(
  const YAML::Node & report,
  const YAML::Node & robot_model,
  const std::vector<std::string> & expected_joint_names = {})
{
  CompiledRunEvidence evidence;
  try {
    if (report["schema_version"].as<int>() != kCompileReportSchemaVersion) {
      throw std::runtime_error("compile_report.yaml schema_version must be 2");
    }
    evidence.compiler_contract_revision = requiredString(
      report, "compiler_contract_revision");
    if (evidence.compiler_contract_revision != kCompilerContractRevision) {
      throw std::runtime_error(
              "compile_report.yaml compiler_contract_revision is unsupported");
    }
    evidence.curobo_revision = normalizedGitCommit(requiredString(report, "curobo_revision"));
    evidence.source_trajectory_sha256 = normalizedHash(
      requiredString(report, "source_trajectory_sha256"));
    evidence.joint_trajectory_sha256 = normalizedHash(
      requiredString(report, "joint_trajectory_sha256"));
    parseAndValidateJointWindingEvidence(
      report, robot_model, expected_joint_names, evidence);
    evidence.duration_s = report["duration_s"].as<double>();
    evidence.point_count = report["point_count"].as<std::size_t>();
    evidence.source_waypoint_count = report["source_waypoint_count"].as<std::size_t>();
    evidence.matched_source_waypoint_count =
      report["matched_source_waypoint_count"].as<std::size_t>();
    evidence.source_duration_s = report["source_duration_s"].as<double>();
    evidence.max_source_time_error_s = report["max_source_time_error_s"].as<double>();
    evidence.uniform_time_scale = report["uniform_time_scale"].as<double>();
    evidence.max_joint_velocity_ratio = report["max_joint_velocity_ratio"].as<double>();
    evidence.max_joint_acceleration_ratio = report["max_joint_acceleration_ratio"].as<double>();
    evidence.max_joint_jerk_ratio = report["max_joint_jerk_ratio"].as<double>();
    evidence.max_tip_position_error_m = report["max_tip_position_error_m"].as<double>();
    evidence.max_tip_orientation_error_deg = report["max_tip_orientation_error_deg"].as<double>();
    evidence.minimum_collision_distance_m = report["minimum_collision_distance_m"].as<double>();
    evidence.max_scene_collision_cost = report["max_scene_collision_cost"].as<double>();
    evidence.max_self_collision_cost = report["max_self_collision_cost"].as<double>();
    evidence.max_joint_bound_cost = report["max_joint_bound_cost"].as<double>();
    evidence.checked_sample_count = report["checked_sample_count"].as<std::size_t>();
    evidence.environment_hashes = parseCompileEnvironmentHashes(report);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "compile_report.yaml or robot_model.yaml has invalid fields: " +
            std::string(error.what()));
  }
  return evidence;
}

void validateCompileEvidenceMetrics(const CompiledRunEvidence & evidence)
{
  if (!std::isfinite(evidence.duration_s) || evidence.duration_s <= 0.0 ||
    evidence.point_count < 2U || evidence.source_waypoint_count < 2U ||
    evidence.matched_source_waypoint_count != evidence.source_waypoint_count ||
    !std::isfinite(evidence.source_duration_s) || evidence.source_duration_s <= 0.0 ||
    std::abs(evidence.duration_s - evidence.source_duration_s) > kTimeTolerance ||
    !std::isfinite(evidence.max_source_time_error_s) ||
    std::abs(evidence.max_source_time_error_s) > 1.0e-12 ||
    !std::isfinite(evidence.uniform_time_scale) ||
    std::abs(evidence.uniform_time_scale - 1.0) > 1.0e-12 ||
    !std::isfinite(evidence.max_joint_velocity_ratio) ||
    !std::isfinite(evidence.max_joint_acceleration_ratio) ||
    !std::isfinite(evidence.max_joint_jerk_ratio) || evidence.max_joint_velocity_ratio < 0.0 ||
    evidence.max_joint_acceleration_ratio < 0.0 || evidence.max_joint_jerk_ratio < 0.0 ||
    evidence.max_joint_velocity_ratio > 1.0 + kTimeTolerance ||
    evidence.max_joint_acceleration_ratio > 1.0 + kTimeTolerance ||
    evidence.max_joint_jerk_ratio > 1.0 + kTimeTolerance ||
    !std::isfinite(evidence.max_tip_position_error_m) ||
    evidence.max_tip_position_error_m < 0.0 ||
    !std::isfinite(evidence.max_tip_orientation_error_deg) ||
    evidence.max_tip_orientation_error_deg < 0.0 ||
    !std::isfinite(evidence.minimum_collision_distance_m) ||
    evidence.minimum_collision_distance_m < 0.0 ||
    !std::isfinite(evidence.max_scene_collision_cost) || evidence.max_scene_collision_cost < 0.0 ||
    !std::isfinite(evidence.max_self_collision_cost) || evidence.max_self_collision_cost < 0.0 ||
    !std::isfinite(evidence.max_joint_bound_cost) || evidence.max_joint_bound_cost < 0.0 ||
    evidence.checked_sample_count != evidence.point_count)
  {
    throw std::runtime_error(
            "compile_report.yaml timing/dynamic metrics are invalid, incomplete, or exceed limits");
  }
}

void requireRecomputedCanonicalEvidenceMatchesReport(
  const CanonicalTrajectoryMetrics & dynamics,
  const CanonicalTipPoseMetrics & tip,
  const CompiledRunEvidence & evidence)
{
  const auto matches = [](
    double actual, double reported, double absolute_tolerance, double relative_tolerance)
    {
      const double tolerance = absolute_tolerance + relative_tolerance *
        std::max(std::abs(actual), std::abs(reported));
      return std::isfinite(actual) && std::isfinite(reported) &&
             std::abs(actual - reported) <= tolerance;
    };
  if (!matches(
      dynamics.max_joint_velocity_ratio, evidence.max_joint_velocity_ratio, 1.0e-6, 1.0e-3) ||
    !matches(
      dynamics.max_joint_acceleration_ratio, evidence.max_joint_acceleration_ratio,
      1.0e-6, 1.0e-3) ||
    !matches(
      dynamics.max_joint_jerk_ratio, evidence.max_joint_jerk_ratio, 1.0e-6, 1.0e-3) ||
    !matches(
      tip.max_tip_position_error_m, evidence.max_tip_position_error_m, 1.0e-6, 1.0e-6) ||
    !matches(
      tip.max_tip_orientation_error_deg, evidence.max_tip_orientation_error_deg,
      1.0e-4, 1.0e-6))
  {
    std::ostringstream message;
    message << std::setprecision(17) <<
      "compile_report.yaml dynamic or source-tip metrics differ from sealed C++ recomputation: " <<
      "velocity_ratio=" << dynamics.max_joint_velocity_ratio << "/" <<
      evidence.max_joint_velocity_ratio << ", acceleration_ratio=" <<
      dynamics.max_joint_acceleration_ratio << "/" << evidence.max_joint_acceleration_ratio <<
      ", jerk_ratio=" << dynamics.max_joint_jerk_ratio << "/" <<
      evidence.max_joint_jerk_ratio << ", tip_position_m=" <<
      tip.max_tip_position_error_m << "/" << evidence.max_tip_position_error_m <<
      ", tip_orientation_deg=" << tip.max_tip_orientation_error_deg << "/" <<
      evidence.max_tip_orientation_error_deg;
    throw std::runtime_error(message.str());
  }
}

[[nodiscard]] std::unordered_map<std::string, std::string> manifestEnvironmentHashes(
  const RecordingManifest & manifest)
{
  static const std::unordered_set<std::string> provenance_keys{
    "environment_tree", "source_trajectory", "joint_trajectory", "curobo_revision"};
  std::unordered_map<std::string, std::string> hashes;
  for (const auto & [key, value] : manifest.hashes) {
    if (provenance_keys.count(key) == 0U) {
      hashes.emplace(key, normalizedHash(value));
    }
  }
  return hashes;
}

void requireCompileEvidenceMatchesValidationNode(
  const CompiledRunEvidence & evidence,
  const YAML::Node & validation,
  const std::string & label)
{
  if (!validation || !validation.IsMap() ||
    validation["schema_version"].as<int>() != kSupportedSchemaVersion ||
    !validation["simulation_completed"].as<bool>() ||
    validation["controller_result"].as<std::string>() != "succeeded")
  {
    throw std::runtime_error(label + " does not prove successful simulation completion");
  }
  const auto require_size = [&validation, &label](const std::string & key, std::size_t expected) {
      const std::size_t actual = validation[key].as<std::size_t>();
      if (actual != expected) {
        throw std::runtime_error(label + " " + key + " differs from compile_report.yaml");
      }
    };
  const auto require_double = [&validation, &label](const std::string & key, double expected) {
      const double actual = validation[key].as<double>();
      const double tolerance = 1.0e-12 * std::max({1.0, std::abs(actual), std::abs(expected)});
      if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(label + " " + key + " differs from compile_report.yaml");
      }
    };
  require_size("point_count", evidence.point_count);
  require_size("source_waypoint_count", evidence.source_waypoint_count);
  require_size("matched_source_waypoint_count", evidence.matched_source_waypoint_count);
  require_size("checked_sample_count", evidence.checked_sample_count);
  require_double("duration_s", evidence.duration_s);
  require_double("source_duration_s", evidence.source_duration_s);
  require_double("max_source_time_error_s", evidence.max_source_time_error_s);
  require_double("uniform_time_scale", evidence.uniform_time_scale);
  require_double("max_joint_velocity_ratio", evidence.max_joint_velocity_ratio);
  require_double("max_joint_acceleration_ratio", evidence.max_joint_acceleration_ratio);
  require_double("max_joint_jerk_ratio", evidence.max_joint_jerk_ratio);
  require_double("max_tip_position_error_m", evidence.max_tip_position_error_m);
  require_double("max_tip_orientation_error_deg", evidence.max_tip_orientation_error_deg);
  require_double("minimum_collision_distance_m", evidence.minimum_collision_distance_m);
  require_double("max_scene_collision_cost", evidence.max_scene_collision_cost);
  require_double("max_self_collision_cost", evidence.max_self_collision_cost);
  require_double("max_joint_bound_cost", evidence.max_joint_bound_cost);
}

void requireCompileEvidenceMatchesManifest(
  const CompiledRunEvidence & evidence,
  const YAML::Node & manifest_root,
  const RecordingManifest & manifest)
{
  if (!manifest_root.IsMap() || requiredString(manifest_root, "recording_id") != manifest.recording_id ||
    requiredString(manifest_root, "environment_id") != manifest.environment_id ||
    requiredString(manifest_root, "trajectory_id") != manifest.trajectory_id)
  {
    throw std::runtime_error("sealed manifest bytes do not match the parsed recording identity");
  }
  const YAML::Node trajectory = manifest_root["trajectory"];
  const YAML::Node validation = manifest_root["validation"];
  if (!trajectory || !trajectory.IsMap() || !validation || !validation.IsMap()) {
    throw std::runtime_error("manifest trajectory/validation compile evidence must be maps");
  }
  const auto require_size = [](const YAML::Node & node, const std::string & key,
      std::size_t expected, const std::string & label) {
      const std::size_t actual = node[key].as<std::size_t>();
      if (actual != expected) {
        throw std::runtime_error(label + " " + key + " differs from compile_report.yaml");
      }
    };
  const auto require_double = [](const YAML::Node & node, const std::string & key,
      double expected, const std::string & label) {
      const double actual = node[key].as<double>();
      const double tolerance = 1.0e-12 * std::max({1.0, std::abs(actual), std::abs(expected)});
      if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(label + " " + key + " differs from compile_report.yaml");
      }
    };

  require_size(trajectory, "point_count", evidence.point_count, "manifest trajectory");
  require_size(
    trajectory, "source_waypoint_count", evidence.source_waypoint_count, "manifest trajectory");
  require_size(
    trajectory, "matched_source_waypoint_count", evidence.matched_source_waypoint_count,
    "manifest trajectory");
  require_double(trajectory, "duration_s", evidence.duration_s, "manifest trajectory");
  require_double(
    trajectory, "source_duration_s", evidence.source_duration_s, "manifest trajectory");
  require_double(
    trajectory, "max_source_time_error_s", evidence.max_source_time_error_s,
    "manifest trajectory");
  require_double(
    trajectory, "uniform_time_scale", evidence.uniform_time_scale, "manifest trajectory");
  require_double(
    trajectory, "max_joint_velocity_ratio", evidence.max_joint_velocity_ratio,
    "manifest trajectory");
  require_double(
    trajectory, "max_joint_acceleration_ratio", evidence.max_joint_acceleration_ratio,
    "manifest trajectory");
  require_double(
    trajectory, "max_joint_jerk_ratio", evidence.max_joint_jerk_ratio,
    "manifest trajectory");

  requireCompileEvidenceMatchesValidationNode(evidence, validation, "manifest validation");
}

[[nodiscard]] std::vector<std::string> requiredSortedUniqueStringSequence(
  const YAML::Node & parent, const std::string & key, const std::string & label)
{
  auto values = requiredStringSequence(parent, key);
  if (!std::is_sorted(values.begin(), values.end()) ||
    std::adjacent_find(values.begin(), values.end()) != values.end())
  {
    throw std::runtime_error(label + " " + key + " must be sorted and contain no duplicates");
  }
  return values;
}

void requireSimulationExecutionContract(
  const YAML::Node & validation_report,
  const YAML::Node & manifest_validation,
  const RecordingManifest & manifest)
{
  if (!validation_report.IsMap() || !manifest_validation.IsMap()) {
    throw std::runtime_error(
            "validation_report.yaml and manifest.validation must both be maps");
  }
  const auto require_matching_string = [&validation_report, &manifest_validation](
      const std::string & key, const std::string & expected) {
      const std::string report_value = requiredString(validation_report, key);
      const std::string manifest_value = requiredString(manifest_validation, key);
      if (report_value != expected || manifest_value != expected || report_value != manifest_value) {
        throw std::runtime_error(
                "simulation execution contract field '" + key + "' is invalid or inconsistent");
      }
    };
  if (validation_report["simulation_execution_contract_revision"].as<int>() !=
    kSimulationExecutionContractRevision ||
    manifest_validation["simulation_execution_contract_revision"].as<int>() !=
    kSimulationExecutionContractRevision)
  {
    throw std::runtime_error("simulation execution contract revision must be 1");
  }
  require_matching_string(
    "speed_scaling_evidence", kRequiredSimulationSpeedScalingEvidence);
  require_matching_string("simulation_controller_name", kRequiredSimulationControllerName);
  require_matching_string("simulation_controller_type", kRequiredSimulationControllerType);
  require_matching_string("simulation_controller_state", "active");
  require_matching_string("simulation_hardware_name", manifest.robot_type);
  require_matching_string("simulation_hardware_type", kRequiredSimulationHardwareType);
  require_matching_string("simulation_hardware_plugin", kRequiredSimulationHardwarePlugin);
  require_matching_string("simulation_hardware_state_label", "active");
  if (validation_report["simulation_hardware_state_id"].as<int>() != kActiveLifecycleStateId ||
    manifest_validation["simulation_hardware_state_id"].as<int>() != kActiveLifecycleStateId)
  {
    throw std::runtime_error("simulation hardware lifecycle state must be active (id=3)");
  }

  std::vector<std::string> expected_command_interfaces;
  std::vector<std::string> expected_state_interfaces;
  expected_command_interfaces.reserve(manifest.joint_names.size());
  expected_state_interfaces.reserve(manifest.joint_names.size() * 2U);
  for (const auto & joint_name : manifest.joint_names) {
    expected_command_interfaces.push_back(joint_name + "/position");
    expected_state_interfaces.push_back(joint_name + "/position");
    expected_state_interfaces.push_back(joint_name + "/velocity");
  }
  std::sort(expected_command_interfaces.begin(), expected_command_interfaces.end());
  std::sort(expected_state_interfaces.begin(), expected_state_interfaces.end());
  const auto require_matching_interfaces = [&validation_report, &manifest_validation](
      const std::string & key, const std::vector<std::string> & expected) {
      const auto report_values = requiredSortedUniqueStringSequence(
        validation_report, key, "validation_report.yaml");
      const auto manifest_values = requiredSortedUniqueStringSequence(
        manifest_validation, key, "manifest.validation");
      if (report_values != expected || manifest_values != expected ||
        report_values != manifest_values)
      {
        throw std::runtime_error(
                "simulation execution interface evidence '" + key +
                "' is invalid or inconsistent");
      }
    };
  require_matching_interfaces(
    "simulation_controller_claimed_interfaces", expected_command_interfaces);
  require_matching_interfaces(
    "simulation_controller_required_command_interfaces", expected_command_interfaces);
  require_matching_interfaces(
    "simulation_controller_required_state_interfaces", expected_state_interfaces);
}

[[nodiscard]] bool yamlNodesExactlyEqual(
  const YAML::Node & left,
  const YAML::Node & right)
{
  if (left.Type() != right.Type()) {
    return false;
  }
  switch (left.Type()) {
    case YAML::NodeType::Undefined:
    case YAML::NodeType::Null:
      return true;
    case YAML::NodeType::Scalar:
      return left.Scalar() == right.Scalar();
    case YAML::NodeType::Sequence:
      if (left.size() != right.size()) {
        return false;
      }
      for (std::size_t index = 0U; index < left.size(); ++index) {
        if (!yamlNodesExactlyEqual(left[index], right[index])) {
          return false;
        }
      }
      return true;
    case YAML::NodeType::Map:
      {
        if (left.size() != right.size()) {
          return false;
        }
        std::set<std::string> left_keys;
        std::set<std::string> right_keys;
        for (const auto & entry : left) {
          if (!entry.first.IsScalar() ||
            !left_keys.insert(entry.first.Scalar()).second)
          {
            return false;
          }
        }
        for (const auto & entry : right) {
          if (!entry.first.IsScalar() ||
            !right_keys.insert(entry.first.Scalar()).second)
          {
            return false;
          }
        }
        if (left_keys != right_keys) {
          return false;
        }
        for (const auto & key : left_keys) {
          if (!yamlNodesExactlyEqual(left[key], right[key])) {
            return false;
          }
        }
        return true;
      }
  }
  return false;
}

void requireBooleanEvidence(
  const YAML::Node & validation,
  const std::string & key,
  bool expected)
{
  const YAML::Node value = validation[key];
  if (!value || !value.IsScalar() || value.as<bool>() != expected) {
    throw std::runtime_error(
            "validation evidence field '" + key + "' does not match recomputed telemetry");
  }
}

void requireSizeEvidence(
  const YAML::Node & validation,
  const std::string & key,
  std::size_t expected)
{
  const YAML::Node value = validation[key];
  if (!value || !value.IsScalar() || value.as<std::size_t>() != expected) {
    throw std::runtime_error(
            "validation evidence field '" + key + "' does not match recomputed telemetry");
  }
}

void requireDoubleEvidence(
  const YAML::Node & validation,
  const std::string & key,
  double expected)
{
  const YAML::Node value = validation[key];
  const double actual = value && value.IsScalar() ? value.as<double>() :
    std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(actual) || !std::isfinite(expected) || actual != expected) {
    throw std::runtime_error(
            "validation evidence field '" + key + "' does not match recomputed telemetry");
  }
}

void requireSamplingEvidence(
  const YAML::Node & validation,
  const std::string & prefix,
  const SamplingValidationMetrics & metrics)
{
  const auto key = [&prefix](const std::string & suffix) {
      return prefix.empty() ? suffix : prefix + "_" + suffix;
    };
  requireSizeEvidence(validation, key("sample_count"), metrics.sample_count);
  requireDoubleEvidence(
    validation, key("first_trajectory_time_s"), metrics.first_trajectory_time_s);
  requireDoubleEvidence(
    validation, key("last_trajectory_time_s"), metrics.last_trajectory_time_s);
  requireDoubleEvidence(
    validation, key("coverage_duration_s"), metrics.coverage_duration_s);
  requireDoubleEvidence(
    validation, key("max_time_interval_s"), metrics.max_time_interval_s);
  requireDoubleEvidence(
    validation, key("max_time_jitter_s"), metrics.max_time_jitter_s);
  requireSizeEvidence(
    validation, key("missing_sample_estimate"), metrics.missing_sample_estimate);
}

void requireSimulationTelemetryEvidence(
  const YAML::Node & validation_report,
  const YAML::Node & manifest_validation,
  const SimulationRecordingMetrics & metrics,
  const SimulationValidationConfig & config,
  const SimulationKinematicsValidationContext & kinematics_context)
{
  if (!validation_report.IsMap() || !manifest_validation.IsMap() ||
    !yamlNodesExactlyEqual(validation_report, manifest_validation))
  {
    throw std::runtime_error(
            "validation_report.yaml must exactly match manifest.validation");
  }

  for (const char * key : {
      "actual_joint_recorded", "controller_state_recorded",
      "joint_frames_recorded", "actual_tip_recorded"})
  {
    requireBooleanEvidence(validation_report, key, true);
  }

  requireDoubleEvidence(validation_report, "sample_dt_s", config.sample_dt_s);
  requireSizeEvidence(
    validation_report, "max_missing_samples_allowed", config.max_missing_samples);
  requireDoubleEvidence(
    validation_report, "max_time_jitter_s_allowed", config.max_time_jitter_s);
  requireBooleanEvidence(
    validation_report, "require_action_success", config.require_action_success);
  requireDoubleEvidence(
    validation_report, "max_quaternion_norm_error_allowed",
    config.max_quaternion_norm_error);
  requireDoubleEvidence(
    validation_report, "max_completion_overrun_s_allowed",
    config.max_completion_overrun_s);
  requireDoubleEvidence(
    validation_report, "max_controller_reference_position_error_rad_allowed",
    config.max_controller_reference_position_error_rad);
  requireDoubleEvidence(
    validation_report, "max_controller_reference_velocity_error_rad_s_allowed",
    config.max_controller_reference_velocity_error_rad_s);
  requireDoubleEvidence(
    validation_report, "max_joint_position_tracking_error_rad_allowed",
    config.max_joint_position_tracking_error_rad);
  requireDoubleEvidence(
    validation_report, "max_joint_velocity_tracking_error_rad_s_allowed",
    config.max_joint_velocity_tracking_error_rad_s);
  requireDoubleEvidence(
    validation_report, "max_actual_controller_position_error_rad_allowed",
    config.max_actual_controller_position_error_rad);
  requireDoubleEvidence(
    validation_report, "max_actual_controller_velocity_error_rad_s_allowed",
    config.max_actual_controller_velocity_error_rad_s);
  requireDoubleEvidence(
    validation_report, "max_frame_fk_position_error_m_allowed",
    config.max_frame_fk_position_error_m);
  requireDoubleEvidence(
    validation_report, "max_frame_fk_orientation_error_deg_allowed",
    config.max_frame_fk_orientation_error_deg);
  requireDoubleEvidence(
    validation_report, "max_tip_fk_position_error_m_allowed",
    config.max_frame_fk_position_error_m);
  requireDoubleEvidence(
    validation_report, "max_tip_fk_orientation_error_deg_allowed",
    config.max_frame_fk_orientation_error_deg);
  requireDoubleEvidence(
    validation_report, "max_tip_source_position_error_m_allowed",
    kinematics_context.max_tip_source_position_error_m);
  requireDoubleEvidence(
    validation_report, "max_tip_source_orientation_error_deg_allowed",
    kinematics_context.max_tip_source_orientation_error_deg);

  requireSamplingEvidence(validation_report, "actual_joint", metrics.actual_joint);
  requireSamplingEvidence(validation_report, "controller", metrics.controller_state);
  requireSizeEvidence(
    validation_report, "joint_frame_sample_count", metrics.joint_frame_sample_count);
  requireSizeEvidence(
    validation_report, "joint_frame_stream_count", metrics.joint_frame_streams.size());
  requireDoubleEvidence(
    validation_report, "max_joint_frame_quaternion_norm_error",
    metrics.max_joint_frame_quaternion_norm_error);

  std::set<std::string> recomputed_frame_ids;
  for (const auto & frame : metrics.joint_frame_streams) {
    if (!recomputed_frame_ids.insert(frame.frame_id).second) {
      throw std::runtime_error("recomputed joint frame telemetry contains duplicate frame ids");
    }
  }
  const std::set<std::string> required_frame_ids(
    kRequiredSimulationFrameNames.begin(), kRequiredSimulationFrameNames.end());
  if (required_frame_ids.size() != kRequiredSimulationFrameNames.size() ||
    recomputed_frame_ids != required_frame_ids)
  {
    throw std::runtime_error(
            "joint frame telemetry must contain exactly the required simulation frame set");
  }

  const YAML::Node reported_frames = validation_report["joint_frame_streams"];
  if (!reported_frames || !reported_frames.IsSequence() ||
    reported_frames.size() != metrics.joint_frame_streams.size())
  {
    throw std::runtime_error(
            "validation joint_frame_streams does not match recomputed telemetry");
  }
  for (std::size_t index = 0U; index < metrics.joint_frame_streams.size(); ++index) {
    const auto & metrics_frame = metrics.joint_frame_streams[index];
    const YAML::Node reported_frame = reported_frames[index];
    if (!reported_frame.IsMap() ||
      requiredString(reported_frame, "frame_id") != metrics_frame.frame_id)
    {
      throw std::runtime_error(
              "validation joint frame id does not match recomputed telemetry");
    }
    requireSamplingEvidence(reported_frame, "", metrics_frame.pose.sampling);
    requireDoubleEvidence(
      reported_frame, "max_quaternion_norm_error",
      metrics_frame.pose.max_quaternion_norm_error);
  }

  requireSamplingEvidence(validation_report, "actual_tip", metrics.actual_tip.sampling);
  requireDoubleEvidence(
    validation_report, "actual_tip_max_quaternion_norm_error",
    metrics.actual_tip.max_quaternion_norm_error);
  requireDoubleEvidence(
    validation_report, "max_speed_scaling_error", metrics.max_speed_scaling_error);
  requireDoubleEvidence(
    validation_report, "max_controller_reference_position_error_rad",
    metrics.max_controller_reference_position_error_rad);
  requireDoubleEvidence(
    validation_report, "max_controller_reference_velocity_error_rad_s",
    metrics.max_controller_reference_velocity_error_rad_s);
  requireDoubleEvidence(
    validation_report, "max_joint_position_tracking_error_rad",
    metrics.max_joint_position_tracking_error_rad);
  requireDoubleEvidence(
    validation_report, "max_joint_velocity_tracking_error_rad_s",
    metrics.max_joint_velocity_tracking_error_rad_s);
  requireDoubleEvidence(
    validation_report, "max_actual_controller_position_error_rad",
    metrics.max_actual_controller_position_error_rad);
  requireDoubleEvidence(
    validation_report, "max_actual_controller_velocity_error_rad_s",
    metrics.max_actual_controller_velocity_error_rad_s);
  requireDoubleEvidence(
    validation_report, "max_frame_fk_position_error_m",
    metrics.max_frame_fk_position_error_m);
  requireDoubleEvidence(
    validation_report, "max_frame_fk_orientation_error_deg",
    metrics.max_frame_fk_orientation_error_deg);
  requireDoubleEvidence(
    validation_report, "max_tip_fk_position_error_m",
    metrics.max_tip_fk_position_error_m);
  requireDoubleEvidence(
    validation_report, "max_tip_fk_orientation_error_deg",
    metrics.max_tip_fk_orientation_error_deg);
  requireDoubleEvidence(
    validation_report, "max_tip_source_position_error_m",
    metrics.max_tip_source_position_error_m);
  requireDoubleEvidence(
    validation_report, "max_tip_source_orientation_error_deg",
    metrics.max_tip_source_orientation_error_deg);

  if (metrics.max_joint_frame_quaternion_norm_error >
    config.max_quaternion_norm_error + 1.0e-12 ||
    metrics.actual_tip.max_quaternion_norm_error >
    config.max_quaternion_norm_error + 1.0e-12 ||
    metrics.max_controller_reference_position_error_rad >
    config.max_controller_reference_position_error_rad + 1.0e-12 ||
    metrics.max_controller_reference_velocity_error_rad_s >
    config.max_controller_reference_velocity_error_rad_s + 1.0e-12 ||
    metrics.max_joint_position_tracking_error_rad >
    config.max_joint_position_tracking_error_rad + 1.0e-12 ||
    metrics.max_joint_velocity_tracking_error_rad_s >
    config.max_joint_velocity_tracking_error_rad_s + 1.0e-12 ||
    metrics.max_actual_controller_position_error_rad >
    config.max_actual_controller_position_error_rad + 1.0e-12 ||
    metrics.max_actual_controller_velocity_error_rad_s >
    config.max_actual_controller_velocity_error_rad_s + 1.0e-12 ||
    metrics.max_frame_fk_position_error_m >
    config.max_frame_fk_position_error_m + 1.0e-12 ||
    metrics.max_frame_fk_orientation_error_deg >
    config.max_frame_fk_orientation_error_deg + 1.0e-12 ||
    metrics.max_tip_fk_position_error_m >
    config.max_frame_fk_position_error_m + 1.0e-12 ||
    metrics.max_tip_fk_orientation_error_deg >
    config.max_frame_fk_orientation_error_deg + 1.0e-12 ||
    metrics.max_tip_source_position_error_m >
    kinematics_context.max_tip_source_position_error_m + 1.0e-12 ||
    metrics.max_tip_source_orientation_error_deg >
    kinematics_context.max_tip_source_orientation_error_deg + 1.0e-12)
  {
    throw std::runtime_error(
            "recomputed simulation telemetry exceeds the sealed recording limits");
  }
}

[[nodiscard]] bool hasField(const std::vector<std::string> & fields, const std::string & field)
{
  return std::find(fields.begin(), fields.end(), field) != fields.end();
}

[[nodiscard]] RecordingManifest parseManifest(const YAML::Node & root)
{
  if (!root.IsMap()) {
    throw std::runtime_error("manifest.yaml root must be a map");
  }

  RecordingManifest manifest;
  try {
    manifest.schema_version = root["schema_version"].as<int>();
  } catch (const YAML::Exception &) {
    throw std::runtime_error("manifest schema_version must be an integer");
  }
  if (manifest.schema_version != kSupportedSchemaVersion) {
    throw std::runtime_error(
            "unsupported recording schema_version " + std::to_string(manifest.schema_version));
  }

  manifest.recording_id = requiredString(root, "recording_id");
  manifest.status = requiredString(root, "status");
  manifest.environment_id = requiredString(root, "environment_id");
  manifest.trajectory_id = requiredString(root, "trajectory_id");
  if (manifest.status != "validated") {
    throw std::runtime_error(
            "recording status is '" + manifest.status + "'; only 'validated' is playable");
  }

  const YAML::Node robot = root["robot"];
  if (!robot || !robot.IsMap()) {
    throw std::runtime_error("manifest robot must be a map");
  }
  manifest.robot_type = requiredString(robot, "type");
  manifest.joint_names = requiredStringSequence(robot, "joint_names");
  if (std::unordered_set<std::string>(manifest.joint_names.begin(), manifest.joint_names.end()).size() !=
    manifest.joint_names.size())
  {
    throw std::runtime_error("manifest robot.joint_names contains duplicates");
  }

  const YAML::Node trajectory = root["trajectory"];
  if (!trajectory || !trajectory.IsMap()) {
    throw std::runtime_error("manifest trajectory must be a map");
  }
  try {
    manifest.duration_s = trajectory["duration_s"].as<double>();
    manifest.point_count = trajectory["point_count"].as<std::size_t>();
  } catch (const YAML::Exception &) {
    throw std::runtime_error("manifest trajectory duration_s and point_count must be numeric");
  }
  if (!std::isfinite(manifest.duration_s) || manifest.duration_s <= 0.0 || manifest.point_count < 2U) {
    throw std::runtime_error("manifest trajectory duration_s/point_count are invalid");
  }
  manifest.interpolation = requiredString(trajectory, "interpolation");
  if (manifest.interpolation != "spline") {
    throw std::runtime_error("recording interpolation must be 'spline'");
  }
  manifest.trajectory_fields = requiredStringSequence(trajectory, "fields");
  for (const char * required : {"position", "velocity", "acceleration"}) {
    if (!hasField(manifest.trajectory_fields, required)) {
      throw std::runtime_error(
              "recording trajectory fields are missing '" + std::string(required) + "'");
    }
  }

  const YAML::Node hashes = root["hashes"];
  if (!hashes || !hashes.IsMap()) {
    throw std::runtime_error("manifest hashes must be a map");
  }
  for (const auto & entry : hashes) {
    if (!entry.first.IsScalar() || !entry.second.IsScalar()) {
      throw std::runtime_error("manifest hashes entries must be scalar key/value pairs");
    }
    manifest.hashes.emplace(entry.first.as<std::string>(), trim(entry.second.as<std::string>()));
  }
  const auto curobo_revision = manifest.hashes.find("curobo_revision");
  if (curobo_revision == manifest.hashes.end()) {
    throw std::runtime_error("manifest hashes is missing required curobo_revision");
  }
  manifest.curobo_revision = normalizedGitCommit(curobo_revision->second);
  return manifest;
}

void verifyUniformTimeScalePolicy(const YAML::Node & planner, double uniform_time_scale);

void verifyCompileValidationPolicy(
  const YAML::Node & planner, const CompiledRunEvidence & evidence);

[[nodiscard]] CompiledRunEvidence validateSealedCompileReport(
  const std::filesystem::path & recording_directory,
  const HashedYamlDocument & manifest_document,
  const RecordingManifest & manifest,
  const std::string & source_trajectory_sha256,
  const std::string & joint_trajectory_sha256,
  const std::unordered_map<std::string, std::string> & sealed_artifact_hashes)
{
  const auto report_path = recording_directory / "compile_report.yaml";
  const auto manifest_path = recording_directory / "manifest.yaml";
  const auto validation_report_path = recording_directory / "validation_report.yaml";
  const auto snapshot_root = recording_directory / "environment_snapshot";
  const auto snapshot_robot_model_path = snapshot_root / "robot/robot_model.yaml";
  const auto required_sealed_hash = [&sealed_artifact_hashes](const std::string & relative) {
      const auto found = sealed_artifact_hashes.find(relative);
      if (found == sealed_artifact_hashes.end()) {
        throw std::runtime_error("sha256sums.txt is missing " + relative);
      }
      return found->second;
    };
  const auto report_document = loadHashedYamlDocument(
    report_path, "sealed compile_report.yaml");
  const auto validation_report_document = loadHashedYamlDocument(
    validation_report_path, "sealed validation_report.yaml");
  const auto robot_model_document = loadHashedYamlDocument(
    snapshot_robot_model_path, "recording environment_snapshot robot_model.yaml");
  if (report_document.sha256 != required_sealed_hash("compile_report.yaml")) {
    throw std::runtime_error(
            "sealed compile_report.yaml bytes do not match sha256sums.txt");
  }
  if (manifest_document.sha256 != required_sealed_hash("manifest.yaml")) {
    throw std::runtime_error("sealed manifest.yaml bytes do not match sha256sums.txt");
  }
  if (validation_report_document.sha256 != required_sealed_hash("validation_report.yaml")) {
    throw std::runtime_error(
            "sealed validation_report.yaml bytes do not match sha256sums.txt");
  }
  const auto expected_environment_hashes = manifestEnvironmentHashes(manifest);
  const auto expected_robot_model_hash = expected_environment_hashes.find(
    "robot/robot_model.yaml");
  if (expected_robot_model_hash == expected_environment_hashes.end() ||
    robot_model_document.sha256 != expected_robot_model_hash->second)
  {
    throw std::runtime_error(
            "recording environment_snapshot robot_model.yaml bytes do not match the manifest");
  }
  const YAML::Node & report = report_document.root;
  const YAML::Node & manifest_root = manifest_document.root;
  const YAML::Node & validation_report = validation_report_document.root;
  const YAML::Node & robot_model = robot_model_document.root;
  if (!report.IsMap() || !manifest_root.IsMap() || !validation_report.IsMap() ||
    !robot_model.IsMap())
  {
    throw std::runtime_error(
            "sealed manifest, compile/validation reports, and snapshot robot model roots must be maps");
  }

  CompiledRunEvidence evidence;
  try {
    if (robot_model["schema_version"].as<int>() != kSupportedSchemaVersion) {
      throw std::runtime_error(
              "recording environment_snapshot robot_model.yaml schema_version must be 1");
    }
    if (requiredString(report, "environment_id") != manifest.environment_id) {
      throw std::runtime_error(
              "sealed compile_report.yaml environment_id does not match the manifest");
    }
    if (requiredString(report, "trajectory_id") != manifest.trajectory_id) {
      throw std::runtime_error(
              "sealed compile_report.yaml trajectory_id does not match the manifest");
    }
    evidence = parseCompileReportEvidence(report, robot_model, manifest.joint_names);
    evidence.compile_report_sha256 = report_document.sha256;
    if (evidence.curobo_revision != manifest.curobo_revision) {
      throw std::runtime_error(
              "sealed compile_report.yaml curobo_revision does not match the manifest");
    }
    if (evidence.source_trajectory_sha256 != source_trajectory_sha256) {
      throw std::runtime_error(
              "sealed compile_report.yaml source_trajectory_sha256 does not match the recording");
    }
    if (evidence.joint_trajectory_sha256 != joint_trajectory_sha256) {
      throw std::runtime_error(
              "sealed compile_report.yaml joint_trajectory_sha256 does not match the recording");
    }
    validateCompileEvidenceMetrics(evidence);
    if (evidence.environment_hashes != expected_environment_hashes) {
      throw std::runtime_error(
              "sealed compile_report.yaml environment_hashes do not exactly match the manifest");
    }
    for (const auto & [relative, expected_hash] : evidence.environment_hashes) {
      const auto snapshot_path = snapshot_root / relative;
      requireNonEmptyArtifactFile(
        snapshot_path, "compile-bound recording environment_snapshot input");
      if (RecordingStore::sha256File(snapshot_path) != expected_hash) {
        throw std::runtime_error(
                "recording environment_snapshot hash mismatch for '" + relative + "'");
      }
    }
    const auto snapshot_planner_path = snapshot_root / "planner.yaml";
    const auto planner_document = loadHashedYamlDocument(
      snapshot_planner_path, "recording environment_snapshot planner.yaml");
    const auto planner_hash = evidence.environment_hashes.find("planner.yaml");
    if (planner_hash == evidence.environment_hashes.end() ||
      planner_document.sha256 != planner_hash->second)
    {
      throw std::runtime_error(
              "recording environment_snapshot planner.yaml bytes do not match compile evidence");
    }
    verifyUniformTimeScalePolicy(planner_document.root, evidence.uniform_time_scale);
    verifyCompileValidationPolicy(planner_document.root, evidence);
    requireCompileEvidenceMatchesManifest(evidence, manifest_root, manifest);
    requireCompileEvidenceMatchesValidationNode(
      evidence, validation_report, "validation_report.yaml");
    requireSimulationExecutionContract(
      validation_report, manifest_root["validation"], manifest);
    if (RecordingStore::sha256File(snapshot_planner_path) != planner_document.sha256) {
      throw std::runtime_error(
              "recording environment_snapshot planner.yaml changed during validation");
    }
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "sealed compile report, manifest, or environment snapshot has invalid fields: " +
            std::string(error.what()));
  }
  if (RecordingStore::sha256File(report_path) != report_document.sha256 ||
    RecordingStore::sha256File(manifest_path) != manifest_document.sha256 ||
    RecordingStore::sha256File(validation_report_path) != validation_report_document.sha256 ||
    RecordingStore::sha256File(snapshot_robot_model_path) != robot_model_document.sha256)
  {
    throw std::runtime_error(
            "sealed manifest, compile/validation report, or snapshot robot model changed during validation");
  }
  return evidence;
}

[[nodiscard]] std::vector<std::string> parseCsvLine(const std::string & line)
{
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (std::size_t index = 0; index < line.size(); ++index) {
    const char character = line[index];
    if (character == '"') {
      if (quoted && index + 1U < line.size() && line[index + 1U] == '"') {
        field.push_back('"');
        ++index;
      } else {
        quoted = !quoted;
      }
    } else if (character == ',' && !quoted) {
      fields.push_back(trim(field));
      field.clear();
    } else {
      field.push_back(character);
    }
  }
  if (quoted) {
    throw std::runtime_error("unterminated quoted CSV field");
  }
  fields.push_back(trim(field));
  return fields;
}

[[nodiscard]] double parseFiniteDouble(
  const std::string & value, std::size_t line_number, const std::string & column)
{
  std::size_t consumed = 0U;
  double parsed = 0.0;
  try {
    parsed = std::stod(value, &consumed);
  } catch (const std::exception &) {
    throw std::runtime_error(
            "invalid numeric value at joint_trajectory.csv line " + std::to_string(line_number) +
            ", column " + column);
  }
  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::runtime_error(
            "non-finite or malformed value at joint_trajectory.csv line " +
            std::to_string(line_number) + ", column " + column);
  }
  return parsed;
}

[[nodiscard]] builtin_interfaces::msg::Duration durationFromSeconds(double seconds)
{
  if (!std::isfinite(seconds) || seconds < 0.0) {
    throw std::runtime_error("trajectory timestamp must be finite and non-negative");
  }
  const long double nanoseconds_ld = static_cast<long double>(seconds) * 1.0e9L;
  constexpr std::int64_t kMaximumDurationNanoseconds =
    static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) * 1000000000LL +
    999999999LL;
  if (nanoseconds_ld > static_cast<long double>(kMaximumDurationNanoseconds)) {
    throw std::runtime_error("trajectory timestamp is outside ROS Duration range");
  }
  const std::int64_t nanoseconds = static_cast<std::int64_t>(std::llround(nanoseconds_ld));
  if (std::abs(nanoseconds_ld - static_cast<long double>(nanoseconds)) > 1.0e-3L) {
    throw std::runtime_error("trajectory timestamp is not exactly representable in ROS nanoseconds");
  }
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  duration.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return duration;
}

[[nodiscard]] double secondsFromDuration(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
}

[[nodiscard]] std::int64_t nanosecondsFromDuration(
  const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<std::int64_t>(duration.sec) * 1000000000LL +
         static_cast<std::int64_t>(duration.nanosec);
}

[[nodiscard]] std::pair<trajectory_msgs::msg::JointTrajectory, std::vector<std::string>>
parseJointTrajectory(
  const std::string & contents,
  const std::vector<std::string> & joint_names)
{
  std::istringstream input(contents);
  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("joint_trajectory.csv is empty");
  }
  if (line.size() >= 3U && static_cast<unsigned char>(line[0]) == 0xefU &&
    static_cast<unsigned char>(line[1]) == 0xbbU && static_cast<unsigned char>(line[2]) == 0xbfU)
  {
    line.erase(0U, 3U);
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }

  std::vector<std::string> expected_header{"time_s"};
  expected_header.reserve(2U + joint_names.size() * 3U);
  for (const auto & name : joint_names) {expected_header.push_back(name + "_pos");}
  for (const auto & name : joint_names) {expected_header.push_back(name + "_vel");}
  for (const auto & name : joint_names) {expected_header.push_back(name + "_acc");}
  expected_header.push_back("phase");
  const auto actual_header = parseCsvLine(line);
  if (actual_header != expected_header) {
    std::ostringstream message;
    message << "joint_trajectory.csv header does not exactly match manifest joint_names";
    throw std::runtime_error(message.str());
  }

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = joint_names;
  std::vector<std::string> phases;
  std::size_t line_number = 1U;
  double previous_time = -1.0;
  while (std::getline(input, line)) {
    ++line_number;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (trim(line).empty()) {
      throw std::runtime_error(
              "joint_trajectory.csv contains an empty row at line " + std::to_string(line_number));
    }
    const auto fields = parseCsvLine(line);
    if (fields.size() != expected_header.size()) {
      throw std::runtime_error(
              "joint_trajectory.csv line " + std::to_string(line_number) +
              " has the wrong number of columns");
    }
    const double time_s = parseFiniteDouble(fields[0], line_number, "time_s");
    if (time_s < 0.0 || (!trajectory.points.empty() && time_s <= previous_time)) {
      throw std::runtime_error("joint trajectory time_s must be non-negative and strictly increasing");
    }
    if (trajectory.points.empty() && std::abs(time_s) > kTimeTolerance) {
      throw std::runtime_error("joint trajectory must start at time_s=0");
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.reserve(joint_names.size());
    point.velocities.reserve(joint_names.size());
    point.accelerations.reserve(joint_names.size());
    std::size_t offset = 1U;
    for (std::size_t index = 0; index < joint_names.size(); ++index) {
      point.positions.push_back(
        parseFiniteDouble(fields[offset + index], line_number, joint_names[index] + "_pos"));
    }
    offset += joint_names.size();
    for (std::size_t index = 0; index < joint_names.size(); ++index) {
      point.velocities.push_back(
        parseFiniteDouble(fields[offset + index], line_number, joint_names[index] + "_vel"));
    }
    offset += joint_names.size();
    for (std::size_t index = 0; index < joint_names.size(); ++index) {
      point.accelerations.push_back(
        parseFiniteDouble(fields[offset + index], line_number, joint_names[index] + "_acc"));
    }
    const std::string phase = trim(fields.back());
    static const std::unordered_set<std::string> valid_phases{"draw", "hold"};
    if (valid_phases.find(phase) == valid_phases.end()) {
      throw std::runtime_error(
              "invalid trajectory phase at joint_trajectory.csv line " + std::to_string(line_number));
    }
    point.time_from_start = durationFromSeconds(time_s);
    trajectory.points.push_back(std::move(point));
    phases.push_back(phase);
    previous_time = time_s;
  }
  if (trajectory.points.size() < 2U) {
    throw std::runtime_error("joint_trajectory.csv requires at least two trajectory points");
  }
  if (phases.front() != "draw") {
    throw std::runtime_error("joint_trajectory.csv first point phase must be draw");
  }
  const auto require_zero_derivatives = [](const auto & point, const std::string & label) {
      const bool velocity_zero = std::all_of(
        point.velocities.begin(), point.velocities.end(), [](double value) {
          return std::abs(value) <= kCanonicalDerivativeTolerance;
        });
      const bool acceleration_zero = std::all_of(
        point.accelerations.begin(), point.accelerations.end(), [](double value) {
          return std::abs(value) <= kCanonicalDerivativeTolerance;
        });
      if (!velocity_zero || !acceleration_zero) {
        throw std::runtime_error(
                "joint_trajectory.csv " + label +
                " must have zero velocity and acceleration");
      }
    };
  require_zero_derivatives(trajectory.points.front(), "first point");
  require_zero_derivatives(trajectory.points.back(), "last point");
  for (std::size_t index = 1U; index < trajectory.points.size(); ++index) {
    if (phases[index] != "hold") {
      continue;
    }
    const auto & previous = trajectory.points[index - 1U];
    const auto & current = trajectory.points[index];
    for (std::size_t joint = 0U; joint < joint_names.size(); ++joint) {
      if (std::abs(current.positions[joint] - previous.positions[joint]) >
        kCanonicalDerivativeTolerance)
      {
        throw std::runtime_error(
                "joint_trajectory.csv hold phase must keep every joint position fixed");
      }
    }
    require_zero_derivatives(previous, "hold boundary");
    require_zero_derivatives(current, "hold sample");
  }
  return {std::move(trajectory), std::move(phases)};
}

struct TimingEvidence
{
  std::vector<std::int64_t> time_ns;
  std::vector<std::string> phases;
};

[[nodiscard]] TimingEvidence parseSourceTimingEvidence(const std::string & contents)
{
  std::istringstream input(contents);
  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("source_tip_trajectory.csv is empty");
  }
  if (line.size() >= 3U && static_cast<unsigned char>(line[0]) == 0xefU &&
    static_cast<unsigned char>(line[1]) == 0xbbU && static_cast<unsigned char>(line[2]) == 0xbfU)
  {
    line.erase(0U, 3U);
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  const std::vector<std::string> expected_header{
    "time_s", "frame_id", "tip_x_m", "tip_y_m", "tip_z_m",
    "tip_qx", "tip_qy", "tip_qz", "tip_qw", "phase"};
  if (parseCsvLine(line) != expected_header) {
    throw std::runtime_error("source_tip_trajectory.csv header is invalid");
  }

  TimingEvidence evidence;
  std::size_t line_number = 1U;
  while (std::getline(input, line)) {
    ++line_number;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (trim(line).empty()) {
      throw std::runtime_error(
              "source_tip_trajectory.csv contains an empty row at line " +
              std::to_string(line_number));
    }
    const auto fields = parseCsvLine(line);
    if (fields.size() != expected_header.size()) {
      throw std::runtime_error(
              "source_tip_trajectory.csv line " + std::to_string(line_number) +
              " has the wrong number of columns");
    }
    const double time_s = parseFiniteDouble(fields.front(), line_number, "time_s");
    const auto duration = durationFromSeconds(time_s);
    const std::int64_t time_ns = nanosecondsFromDuration(duration);
    if (!evidence.time_ns.empty() && time_ns <= evidence.time_ns.back()) {
      throw std::runtime_error(
              "source waypoint time_s must be strictly increasing");
    }
    const std::string phase = fields.back();
    if (phase != "draw" && phase != "hold") {
      throw std::runtime_error("source waypoint phase must be draw or hold");
    }
    evidence.time_ns.push_back(time_ns);
    evidence.phases.push_back(phase);
  }
  if (evidence.time_ns.size() < 2U || evidence.time_ns.front() != 0LL) {
    throw std::runtime_error(
            "source_tip_trajectory.csv must contain at least two points and start at time_s=0");
  }
  return evidence;
}

[[nodiscard]] TimingEvidence parseJointTimingEvidence(const std::string & contents)
{
  std::istringstream input(contents);
  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("joint_trajectory.csv is empty");
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  const auto header = parseCsvLine(line);
  if (header.size() < 3U || header.front() != "time_s" || header.back() != "phase") {
    throw std::runtime_error("joint_trajectory.csv timing header is invalid");
  }
  TimingEvidence evidence;
  std::size_t line_number = 1U;
  while (std::getline(input, line)) {
    ++line_number;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const auto fields = parseCsvLine(line);
    if (fields.size() != header.size()) {
      throw std::runtime_error(
              "joint_trajectory.csv line " + std::to_string(line_number) +
              " has the wrong number of columns");
    }
    const auto duration = durationFromSeconds(
      parseFiniteDouble(fields.front(), line_number, "time_s"));
    const std::int64_t time_ns = nanosecondsFromDuration(duration);
    if (!evidence.time_ns.empty() && time_ns <= evidence.time_ns.back()) {
      throw std::runtime_error("joint trajectory time_s must be strictly increasing");
    }
    evidence.time_ns.push_back(time_ns);
    evidence.phases.push_back(fields.back());
  }
  if (evidence.time_ns.size() < 2U || evidence.time_ns.front() != 0LL) {
    throw std::runtime_error(
            "joint_trajectory.csv must contain at least two points and start at time_s=0");
  }
  return evidence;
}

[[nodiscard]] std::size_t verifyAuthoritativeTiming(
  const TimingEvidence & source,
  const TimingEvidence & joint)
{
  if (source.time_ns.back() != joint.time_ns.back()) {
    throw std::runtime_error(
            "joint trajectory duration does not equal authoritative source duration");
  }
  std::size_t joint_index = 0U;
  for (std::size_t source_index = 0U; source_index < source.time_ns.size(); ++source_index) {
    while (joint_index < joint.time_ns.size() &&
      joint.time_ns[joint_index] < source.time_ns[source_index])
    {
      ++joint_index;
    }
    if (joint_index >= joint.time_ns.size() ||
      joint.time_ns[joint_index] != source.time_ns[source_index])
    {
      throw std::runtime_error(
              "joint trajectory does not contain every authoritative source time exactly");
    }
    if (joint.phases[joint_index] != source.phases[source_index]) {
      throw std::runtime_error(
              "joint trajectory phase differs at an authoritative source time");
    }
    ++joint_index;
  }
  return source.time_ns.size();
}

void requireManifestNominalDtMatchesCanonicalTrajectory(
  const YAML::Node & manifest_root,
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (trajectory.points.size() < 2U) {
    throw std::runtime_error(
            "canonical trajectory requires at least two points for nominal_dt_s validation");
  }
  try {
    const YAML::Node trajectory_node = manifest_root["trajectory"];
    const double reported_nominal_dt_s = trajectory_node["nominal_dt_s"].as<double>();
    const double actual_nominal_dt_s =
      secondsFromDuration(trajectory.points[1U].time_from_start) -
      secondsFromDuration(trajectory.points[0U].time_from_start);
    const double tolerance = 1.0e-12 *
      std::max({1.0, std::abs(reported_nominal_dt_s), std::abs(actual_nominal_dt_s)});
    if (!std::isfinite(reported_nominal_dt_s) || reported_nominal_dt_s <= 0.0 ||
      !std::isfinite(actual_nominal_dt_s) || actual_nominal_dt_s <= 0.0 ||
      std::abs(reported_nominal_dt_s - actual_nominal_dt_s) > tolerance)
    {
      throw std::runtime_error(
              "manifest trajectory nominal_dt_s does not match the canonical first interval");
    }
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "manifest trajectory nominal_dt_s is missing or invalid: " +
            std::string(error.what()));
  }
}

[[nodiscard]] bool isSafeIdentifier(const std::string & value)
{
  return !value.empty() && value != "." && value != ".." &&
         value.find('/') == std::string::npos && value.find('\\') == std::string::npos;
}

[[nodiscard]] bool pathIsWithin(
  const std::filesystem::path & child, const std::filesystem::path & parent)
{
  auto child_it = child.begin();
  for (auto parent_it = parent.begin(); parent_it != parent.end(); ++parent_it, ++child_it) {
    if (child_it == child.end() || *child_it != *parent_it) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool hasParentReference(const std::filesystem::path & path)
{
  return std::any_of(path.begin(), path.end(), [](const auto & component) {
      return component == "..";
    });
}

void requireCompleteRecordingArtifact(const std::filesystem::path & recording_directory)
{
  for (const char * relative : kRequiredRecordingFiles) {
    requireNonEmptyArtifactFile(
      recording_directory / relative, "required recording file '" + std::string(relative) + "'");
  }

  const auto snapshot = recording_directory / "environment_snapshot";
  const auto snapshot_status = std::filesystem::symlink_status(snapshot);
  if (std::filesystem::is_symlink(snapshot_status) || !std::filesystem::is_directory(snapshot_status)) {
    throw std::runtime_error(
            "recording environment_snapshot is missing, not a directory, or is a symlink");
  }
  for (const char * relative : kRequiredEnvironmentFiles) {
    requireNonEmptyArtifactFile(
      snapshot / relative,
      "required environment snapshot file '" + std::string(relative) + "'");
  }
}

[[nodiscard]] std::unordered_map<std::string, std::string> verifyArtifactChecksums(
  const std::filesystem::path & recording_directory,
  const std::filesystem::path & checksum_file)
{
  requireNonEmptyArtifactFile(checksum_file, "recording checksum file");

  std::set<std::string> artifact_files;
  for (const auto & entry : std::filesystem::recursive_directory_iterator(recording_directory)) {
    const auto status = entry.symlink_status();
    if (std::filesystem::is_symlink(status)) {
      throw std::runtime_error(
              "recording artifacts must not contain symlinks: " + entry.path().string());
    }
    if (std::filesystem::is_directory(status)) {
      continue;
    }
    if (!std::filesystem::is_regular_file(status)) {
      throw std::runtime_error(
              "recording artifacts may contain only directories and regular files: " +
              entry.path().string());
    }
    const std::string relative =
      entry.path().lexically_relative(recording_directory).generic_string();
    if (relative != "sha256sums.txt") {
      artifact_files.insert(relative);
    }
  }

  std::ifstream input(checksum_file);
  if (!input) {
    throw std::runtime_error("failed to open " + checksum_file.string());
  }
  std::set<std::string> checksummed_files;
  std::unordered_map<std::string, std::string> declared_hashes;
  std::string line;
  std::size_t line_number = 0U;
  while (std::getline(input, line)) {
    ++line_number;
    line = trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    if (line.size() < 66U) {
      throw std::runtime_error("malformed sha256sums.txt line " + std::to_string(line_number));
    }
    const std::string expected = normalizedHash(line.substr(0U, 64U));
    std::string relative_name = trim(line.substr(64U));
    if (!relative_name.empty() && relative_name[0] == '*') {
      relative_name.erase(0U, 1U);
    }
    const std::filesystem::path relative = std::filesystem::path(relative_name).lexically_normal();
    const std::string normalized_relative = relative.generic_string();
    if (relative.is_absolute() || relative.empty() || relative == "." ||
      hasParentReference(relative) || relative_name.find('\\') != std::string::npos ||
      normalized_relative == "sha256sums.txt")
    {
      throw std::runtime_error("unsafe path in sha256sums.txt line " + std::to_string(line_number));
    }
    if (artifact_files.count(normalized_relative) == 0U) {
      throw std::runtime_error(
              "sha256sums.txt references a missing or non-regular artifact: " + normalized_relative);
    }
    if (!checksummed_files.insert(normalized_relative).second) {
      throw std::runtime_error(
              "sha256sums.txt contains a duplicate path: " + normalized_relative);
    }
    declared_hashes.emplace(normalized_relative, expected);
    const auto file_path = recording_directory / relative;
    const std::string actual = RecordingStore::sha256File(file_path);
    if (actual != expected) {
      throw std::runtime_error("SHA-256 mismatch for " + normalized_relative);
    }
  }
  if (checksummed_files != artifact_files) {
    std::vector<std::string> missing;
    std::set_difference(
      artifact_files.begin(), artifact_files.end(), checksummed_files.begin(), checksummed_files.end(),
      std::back_inserter(missing));
    throw std::runtime_error(
            "sha256sums.txt does not protect every recording file; missing: " +
            (missing.empty() ? std::string("<unknown>") : missing.front()));
  }
  return declared_hashes;
}

void validateSealedSimulationTelemetry(
  const std::filesystem::path & recording_directory,
  const YAML::Node & manifest_root,
  const trajectory_msgs::msg::JointTrajectory & commanded_trajectory,
  const SimulationKinematicsValidationContext & kinematics_context,
  const std::unordered_map<std::string, std::string> & sealed_artifact_hashes)
{
  const auto required_sealed_hash = [&sealed_artifact_hashes](const std::string & relative) {
      const auto found = sealed_artifact_hashes.find(relative);
      if (found == sealed_artifact_hashes.end()) {
        throw std::runtime_error("sha256sums.txt is missing " + relative);
      }
      return found->second;
    };

  const auto validation_report_path = recording_directory / "validation_report.yaml";
  const auto recording_config_path =
    recording_directory / "environment_snapshot/recording.yaml";
  const auto validation_document = loadHashedYamlDocument(
    validation_report_path, "sealed validation_report.yaml");
  const auto recording_config_document = loadHashedYamlDocument(
    recording_config_path, "sealed environment_snapshot recording.yaml");
  if (validation_document.sha256 != required_sealed_hash("validation_report.yaml") ||
    recording_config_document.sha256 !=
    required_sealed_hash("environment_snapshot/recording.yaml"))
  {
    throw std::runtime_error(
            "sealed validation report or recording configuration does not match sha256sums.txt");
  }

  const SimulationRecordingPaths paths{
    recording_directory / "sim_actual_joint.csv",
    recording_directory / "sim_controller_state.csv",
    recording_directory / "joint_frames.csv",
    recording_directory / "sim_actual_tip.csv"};
  const std::array<std::pair<const char *, std::filesystem::path>, 4> telemetry_files{{
      {"sim_actual_joint.csv", paths.actual_joint},
      {"sim_controller_state.csv", paths.controller_state},
      {"joint_frames.csv", paths.joint_frames},
      {"sim_actual_tip.csv", paths.actual_tip}}};
  for (const auto & [relative, path] : telemetry_files) {
    if (RecordingStore::sha256File(path) != required_sealed_hash(relative)) {
      throw std::runtime_error(
              "sealed simulation telemetry does not match sha256sums.txt: " +
              std::string(relative));
    }
  }

  const auto config = SimulationRecordingValidator::loadConfig(recording_config_path);
  const auto metrics = SimulationRecordingValidator::validate(
    paths, commanded_trajectory, true, config, kinematics_context);
  try {
    requireSimulationTelemetryEvidence(
      validation_document.root, manifest_root["validation"], metrics, config,
      kinematics_context);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "sealed simulation validation evidence has missing or invalid fields: " +
            std::string(error.what()));
  }

  for (const auto & [relative, path] : telemetry_files) {
    if (RecordingStore::sha256File(path) != required_sealed_hash(relative)) {
      throw std::runtime_error(
              "sealed simulation telemetry changed during validation: " +
              std::string(relative));
    }
  }
  if (RecordingStore::sha256File(validation_report_path) != validation_document.sha256 ||
    RecordingStore::sha256File(recording_config_path) != recording_config_document.sha256)
  {
    throw std::runtime_error(
            "sealed validation report or recording configuration changed during validation");
  }
}

void verifyEnvironmentHashes(
  const RecordingManifest & manifest, const std::filesystem::path & environment_directory)
{
  if (!std::filesystem::is_directory(environment_directory)) {
    throw std::runtime_error(
            "environment directory does not exist: " + environment_directory.string());
  }
  for (const char * relative_path : kRequiredEnvironmentFiles) {
    const auto hash_it = manifest.hashes.find(relative_path);
    if (hash_it == manifest.hashes.end()) {
      throw std::runtime_error("manifest hashes is missing required environment file '" +
              std::string(relative_path) + "'");
    }
    const auto file_path = environment_directory / relative_path;
    requireRegularFile(file_path, "current environment file");
    if (RecordingStore::sha256File(file_path) != normalizedHash(hash_it->second)) {
      throw std::runtime_error(
              "current environment does not match recording hash for '" +
              std::string(relative_path) + "'");
    }
  }

  // Verify any additional path-like environment inputs (for example meshes) that were captured
  // by the compiler. Provenance-only values are deliberately not interpreted as file paths.
  static const std::unordered_set<std::string> provenance_keys{
    "environment_tree", "source_trajectory", "joint_trajectory", "curobo_revision"};
  for (const auto & [key, expected_hash] : manifest.hashes) {
    if (provenance_keys.count(key) != 0U) {
      continue;
    }
    const auto relative = std::filesystem::path(key).lexically_normal();
    if (relative.is_absolute() || relative.empty() || relative.string().rfind("..", 0U) == 0U) {
      throw std::runtime_error("unsafe environment path in manifest hashes: " + key);
    }
    const auto file_path = environment_directory / relative;
    requireRegularFile(file_path, "hashed current environment input");
    if (RecordingStore::sha256File(file_path) != normalizedHash(expected_hash)) {
      throw std::runtime_error("current environment hash mismatch for '" + key + "'");
    }
  }

  const auto environment_path = environment_directory / "environment.yaml";
  const auto robot_model_path = environment_directory / "robot/robot_model.yaml";
  const auto environment_document = loadHashedYamlDocument(
    environment_path, "current environment.yaml");
  const auto robot_model_document = loadHashedYamlDocument(
    robot_model_path, "current robot_model.yaml");
  if (environment_document.sha256 != normalizedHash(manifest.hashes.at("environment.yaml")) ||
    robot_model_document.sha256 != normalizedHash(
      manifest.hashes.at("robot/robot_model.yaml")))
  {
    throw std::runtime_error(
            "parsed current environment identity bytes do not match the recording manifest");
  }
  try {
    const YAML::Node & environment_config = environment_document.root;
    const auto configured_environment_id = environment_config["environment_id"].as<std::string>();
    if (configured_environment_id != manifest.environment_id) {
      throw std::runtime_error(
              "current environment.yaml environment_id does not match recording manifest");
    }
    const YAML::Node & robot_model = robot_model_document.root;
    const auto configured_type = robot_model["robot"]["type"].as<std::string>();
    const auto configured_joints =
      robot_model["robot"]["joint_names"].as<std::vector<std::string>>();
    if (configured_type != manifest.robot_type) {
      throw std::runtime_error("current environment robot type does not match recording manifest");
    }
    if (configured_joints != manifest.joint_names) {
      throw std::runtime_error("current environment joint_names do not match recording manifest");
    }
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "failed to validate current robot_model.yaml against recording: " +
            std::string(error.what()));
  }
  if (RecordingStore::sha256File(environment_path) != environment_document.sha256 ||
    RecordingStore::sha256File(robot_model_path) != robot_model_document.sha256)
  {
    throw std::runtime_error(
            "current environment identity files changed during recording validation");
  }
}

[[nodiscard]] std::unordered_map<std::string, std::string> currentEnvironmentHashes(
  const std::filesystem::path & environment_directory)
{
  if (!std::filesystem::is_directory(environment_directory)) {
    throw std::runtime_error(
            "compile evidence environment directory does not exist: " +
            environment_directory.string());
  }
  const auto canonical_environment = std::filesystem::canonical(environment_directory);
  std::unordered_map<std::string, std::string> hashes;
  for (const char * relative : kRequiredEnvironmentFiles) {
    const auto path = canonical_environment / relative;
    requireNonEmptyArtifactFile(path, "compile evidence environment input");
    hashes.emplace(relative, RecordingStore::sha256File(path));
  }
  for (const char * relative_directory : kEnvironmentTreeDirectories) {
    const auto directory = canonical_environment / relative_directory;
    const auto directory_status = std::filesystem::symlink_status(directory);
    if (std::filesystem::is_symlink(directory_status)) {
      throw std::runtime_error(
              "compile evidence environment tree directory must not be a symlink: " +
              directory.string());
    }
    if (std::filesystem::is_directory(directory_status)) {
      for (const auto & entry : std::filesystem::recursive_directory_iterator(directory)) {
        const auto status = entry.symlink_status();
        if (std::filesystem::is_symlink(status)) {
          throw std::runtime_error(
                  "compile evidence environment tree must not contain symlinks: " +
                  entry.path().string());
        }
        if (std::filesystem::is_directory(status)) {
          continue;
        }
        if (!std::filesystem::is_regular_file(status)) {
          throw std::runtime_error(
                  "compile evidence environment tree contains a non-regular file: " +
                  entry.path().string());
        }
        const auto relative = entry.path().lexically_relative(canonical_environment).generic_string();
        hashes.emplace(relative, RecordingStore::sha256File(entry.path()));
      }
    }
  }
  return hashes;
}

void verifyUniformTimeScalePolicy(
  const YAML::Node & planner,
  double uniform_time_scale)
{
  try {
    const YAML::Node limits = planner["limits"];
    const bool allow_uniform_time_scale = limits["allow_uniform_time_scale"].as<bool>();
    const std::string on_violation = limits["on_violation"].as<std::string>();
    const double maximum_scale = limits["max_uniform_time_scale"].as<double>();
    if (allow_uniform_time_scale || on_violation != "reject" ||
      !std::isfinite(maximum_scale) || std::abs(maximum_scale - 1.0) > kTimeTolerance)
    {
      throw std::runtime_error(
              "current planner policy must reject violations and forbid all time scaling");
    }
    if (std::abs(uniform_time_scale - 1.0) > kTimeTolerance) {
      throw std::runtime_error(
              "compile report uniform_time_scale must be exactly 1.0");
    }
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "failed to validate compile report against planner time-scale policy: " +
            std::string(error.what()));
  }
}

void verifyCompileValidationPolicy(
  const YAML::Node & planner,
  const CompiledRunEvidence & evidence)
{
  try {
    const YAML::Node validation = planner["validation"];
    const double position_tolerance = validation["tip_position_tolerance_m"].as<double>();
    const double orientation_tolerance = validation["tip_orientation_tolerance_deg"].as<double>();
    const double minimum_clearance = validation["minimum_collision_distance_m"].as<double>();
    if (!std::isfinite(position_tolerance) || position_tolerance < 0.0 ||
      !std::isfinite(orientation_tolerance) || orientation_tolerance < 0.0 ||
      !std::isfinite(minimum_clearance) || minimum_clearance < 0.0)
    {
      throw std::runtime_error("planner compile-validation thresholds are invalid");
    }
    if (evidence.checked_sample_count != evidence.point_count) {
      throw std::runtime_error(
              "compile report checked_sample_count does not cover every trajectory point");
    }
    if (evidence.max_tip_position_error_m > position_tolerance + kTimeTolerance) {
      throw std::runtime_error("compile report tip position error exceeds current planner tolerance");
    }
    if (evidence.max_tip_orientation_error_deg > orientation_tolerance + kTimeTolerance) {
      throw std::runtime_error(
              "compile report tip orientation error exceeds current planner tolerance");
    }
    if (evidence.minimum_collision_distance_m + kTimeTolerance < minimum_clearance) {
      throw std::runtime_error(
              "compile report minimum collision distance is below current planner threshold");
    }
    if (evidence.max_scene_collision_cost > kCuroboFeasibilityCostTolerance ||
      evidence.max_self_collision_cost > kCuroboFeasibilityCostTolerance ||
      evidence.max_joint_bound_cost > kCuroboFeasibilityCostTolerance)
    {
      throw std::runtime_error(
              "compile report contains non-zero cuRobo scene/self/bound feasibility cost");
    }
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "failed to validate compile report against planner validation policy: " +
            std::string(error.what()));
  }
}

}  // namespace

RecordingStore::RecordingStore(std::filesystem::path environments_root)
: environments_root_(std::filesystem::absolute(std::move(environments_root)).lexically_normal())
{
  // Keep startup independent of mount/order timing. loadValidated() still requires and
  // canonicalizes the root before any recording is accepted.
  if (std::filesystem::is_directory(environments_root_)) {
    environments_root_ = std::filesystem::canonical(environments_root_);
  }
}

RecordingArtifact RecordingStore::loadValidated(
  const std::string & environment_id,
  const std::string & recording_id,
  const std::string & expected_robot_type,
  const std::vector<std::string> & expected_joint_names) const
{
  if (!isSafeIdentifier(environment_id) || !isSafeIdentifier(recording_id)) {
    throw std::runtime_error("environment_id and recording_id must be simple path-safe identifiers");
  }
  if (!std::filesystem::is_directory(environments_root_)) {
    throw std::runtime_error("environments root does not exist: " + environments_root_.string());
  }
  const auto canonical_root = std::filesystem::canonical(environments_root_);
  const auto environment_directory = std::filesystem::canonical(canonical_root / environment_id);
  if (!pathIsWithin(environment_directory, canonical_root)) {
    throw std::runtime_error("environment directory escapes environments_root");
  }
  const auto recording_directory =
    std::filesystem::canonical(environment_directory / "recordings" / recording_id);
  if (!pathIsWithin(recording_directory, environment_directory / "recordings")) {
    throw std::runtime_error("recording directory escapes the selected environment");
  }
  RecordingLoadOptions options;
  options.environment_directory = environment_directory;
  options.expected_environment_id = environment_id;
  options.expected_robot_type = expected_robot_type;
  options.expected_joint_names = expected_joint_names;
  return loadValidatedDirectory(recording_directory, options);
}

RecordingArtifact RecordingStore::loadValidatedDirectory(
  const std::filesystem::path & recording_directory,
  const RecordingLoadOptions & options)
{
  if (!std::filesystem::is_directory(recording_directory)) {
    throw std::runtime_error("recording directory does not exist: " + recording_directory.string());
  }
  const auto canonical_directory = std::filesystem::canonical(recording_directory);
  const auto manifest_path = canonical_directory / "manifest.yaml";
  const auto source_path = canonical_directory / "source_tip_trajectory.csv";
  const auto trajectory_path = canonical_directory / "joint_trajectory.csv";
  const auto checksum_path = canonical_directory / "sha256sums.txt";
  requireCompleteRecordingArtifact(canonical_directory);
  const auto sealed_artifact_hashes = verifyArtifactChecksums(
    canonical_directory, checksum_path);
  const std::string checksum_file_sha256 = sha256File(checksum_path);
  const auto manifest_document = loadHashedYamlDocument(manifest_path, "sealed manifest.yaml");
  const auto sealed_manifest_hash = sealed_artifact_hashes.find("manifest.yaml");
  if (sealed_manifest_hash == sealed_artifact_hashes.end() ||
    manifest_document.sha256 != sealed_manifest_hash->second)
  {
    throw std::runtime_error("sealed manifest.yaml bytes do not match sha256sums.txt");
  }

  RecordingArtifact artifact;
  artifact.directory = canonical_directory;
  artifact.manifest = parseManifest(manifest_document.root);
  if (artifact.manifest.recording_id != canonical_directory.filename().string()) {
    throw std::runtime_error("manifest recording_id does not match its directory name");
  }
  if (!options.expected_environment_id.empty() &&
    artifact.manifest.environment_id != options.expected_environment_id)
  {
    throw std::runtime_error("manifest environment_id does not match the requested environment");
  }
  if (!options.expected_robot_type.empty() && artifact.manifest.robot_type != options.expected_robot_type) {
    throw std::runtime_error("manifest robot type does not match the configured robot type");
  }
  if (!options.expected_joint_names.empty() &&
    artifact.manifest.joint_names != options.expected_joint_names)
  {
    throw std::runtime_error("manifest joint_names do not exactly match the configured controller order");
  }

  verifyEnvironmentHashes(artifact.manifest, options.environment_directory);

  const auto source_contents = loadHashedFileContents(
    source_path, "sealed source_tip_trajectory.csv");
  const auto joint_contents = loadHashedFileContents(
    trajectory_path, "sealed joint_trajectory.csv");
  const auto sealed_source_hash = sealed_artifact_hashes.find("source_tip_trajectory.csv");
  const auto sealed_joint_hash = sealed_artifact_hashes.find("joint_trajectory.csv");
  if (sealed_source_hash == sealed_artifact_hashes.end() ||
    source_contents.sha256 != sealed_source_hash->second ||
    sealed_joint_hash == sealed_artifact_hashes.end() ||
    joint_contents.sha256 != sealed_joint_hash->second)
  {
    throw std::runtime_error(
            "sealed source/joint trajectory bytes do not match sha256sums.txt");
  }
  const std::string & actual_source_hash = source_contents.sha256;
  const auto source_hash = artifact.manifest.hashes.find("source_trajectory");
  if (source_hash == artifact.manifest.hashes.end() ||
    actual_source_hash != normalizedHash(source_hash->second))
  {
    throw std::runtime_error("manifest source_trajectory SHA-256 does not match the artifact");
  }
  const std::string & actual_joint_hash = joint_contents.sha256;
  const auto joint_hash = artifact.manifest.hashes.find("joint_trajectory");
  if (joint_hash == artifact.manifest.hashes.end() ||
    actual_joint_hash != normalizedHash(joint_hash->second))
  {
    throw std::runtime_error("manifest joint_trajectory SHA-256 does not match the artifact");
  }
  const auto compile_evidence = validateSealedCompileReport(
    canonical_directory, manifest_document, artifact.manifest, actual_source_hash,
    actual_joint_hash, sealed_artifact_hashes);

  auto parsed = parseJointTrajectory(joint_contents.bytes, artifact.manifest.joint_names);
  artifact.trajectory = std::move(parsed.first);
  artifact.phases = std::move(parsed.second);
  const auto source_timing = parseSourceTimingEvidence(source_contents.bytes);
  const auto joint_timing = parseJointTimingEvidence(joint_contents.bytes);
  const std::size_t matched_source_count = verifyAuthoritativeTiming(
    source_timing, joint_timing);
  if (source_timing.time_ns.size() != compile_evidence.source_waypoint_count ||
    matched_source_count != compile_evidence.matched_source_waypoint_count ||
    source_timing.time_ns.back() !=
    nanosecondsFromDuration(durationFromSeconds(compile_evidence.source_duration_s)) ||
    joint_timing.time_ns.size() != compile_evidence.point_count ||
    joint_timing.time_ns.back() !=
    nanosecondsFromDuration(durationFromSeconds(compile_evidence.duration_s)))
  {
    throw std::runtime_error(
            "sealed compile report authoritative timing evidence does not match source/joint CSV");
  }
  const auto snapshot_environment = canonical_directory / "environment_snapshot";
  const auto robot_validation_model = loadRobotValidationModel(
    snapshot_environment, artifact.manifest.joint_names);
  const auto canonical_dynamics = validateCanonicalTrajectoryDynamics(
    artifact.trajectory, robot_validation_model);
  const auto kinematics_context = SimulationRecordingValidator::loadKinematicsContext(
    snapshot_environment, source_path, artifact.manifest.joint_names);
  const auto canonical_tip = SimulationRecordingValidator::validateCanonicalSourcePoses(
    artifact.trajectory, kinematics_context);
  requireRecomputedCanonicalEvidenceMatchesReport(
    canonical_dynamics, canonical_tip, compile_evidence);
  if (artifact.trajectory.points.size() != artifact.manifest.point_count) {
    throw std::runtime_error("joint trajectory point count does not match manifest");
  }
  const double actual_duration = secondsFromDuration(artifact.trajectory.points.back().time_from_start);
  if (std::abs(actual_duration - artifact.manifest.duration_s) > kTimeTolerance) {
    throw std::runtime_error("joint trajectory duration does not match manifest");
  }
  requireManifestNominalDtMatchesCanonicalTrajectory(
    manifest_document.root, artifact.trajectory);
  validateSealedSimulationTelemetry(
    canonical_directory, manifest_document.root, artifact.trajectory, kinematics_context,
    sealed_artifact_hashes);
  if (sha256File(source_path) != source_contents.sha256 ||
    sha256File(trajectory_path) != joint_contents.sha256)
  {
    throw std::runtime_error("sealed source/joint trajectory changed during playback validation");
  }
  const auto final_artifact_hashes = verifyArtifactChecksums(
    canonical_directory, checksum_path);
  if (final_artifact_hashes != sealed_artifact_hashes ||
    sha256File(checksum_path) != checksum_file_sha256)
  {
    throw std::runtime_error(
            "recording checksum or artifact directory closure changed during playback validation");
  }
  return artifact;
}

CompiledRunEvidence RecordingStore::validateCompiledRun(
  const std::filesystem::path & runs_root,
  const std::filesystem::path & environment_directory,
  const std::filesystem::path & source_trajectory_path,
  const std::filesystem::path & joint_trajectory_path,
  const std::string & expected_environment_id,
  const std::string & expected_trajectory_id)
{
  if (!isSafeIdentifier(expected_environment_id) || !isSafeIdentifier(expected_trajectory_id)) {
    throw std::runtime_error(
            "compiled run environment_id and trajectory_id must be path-safe identifiers");
  }
  if (!std::filesystem::is_directory(runs_root)) {
    throw std::runtime_error("controlled runs root does not exist: " + runs_root.string());
  }
  const auto canonical_runs_root = std::filesystem::canonical(runs_root);
  requireNonEmptyArtifactFile(source_trajectory_path, "compiled run source trajectory");
  requireNonEmptyArtifactFile(joint_trajectory_path, "compiled run joint trajectory");
  const auto canonical_source = std::filesystem::canonical(source_trajectory_path);
  const auto canonical_trajectory = std::filesystem::canonical(joint_trajectory_path);
  const auto source_contents = loadHashedFileContents(
    canonical_source, "compiled run source trajectory");
  const auto joint_contents = loadHashedFileContents(
    canonical_trajectory, "compiled run joint trajectory");
  if (sha256File(canonical_source) != source_contents.sha256 ||
    sha256File(canonical_trajectory) != joint_contents.sha256)
  {
    throw std::runtime_error(
            "compiled source or joint trajectory changed while being loaded");
  }
  if (!pathIsWithin(canonical_source, canonical_runs_root) ||
    !pathIsWithin(canonical_trajectory, canonical_runs_root))
  {
    throw std::runtime_error(
            "BuildRecording inputs must be contained by the configured controlled runs root");
  }
  if (canonical_source.filename() != "source_tip_trajectory.csv" ||
    canonical_trajectory.filename() != "joint_trajectory.csv")
  {
    throw std::runtime_error(
            "compiled run inputs must use source_tip_trajectory.csv and joint_trajectory.csv");
  }
  const auto run_directory = canonical_source.parent_path();
  if (canonical_trajectory.parent_path() != run_directory ||
    run_directory.parent_path() != canonical_runs_root)
  {
    throw std::runtime_error(
            "BuildRecording inputs must be sibling files in one direct child of runs/");
  }
  const std::string expected_prefix = expected_trajectory_id + "__compiled__";
  if (run_directory.filename().string().rfind(expected_prefix, 0U) != 0U) {
    throw std::runtime_error(
            "compiled run directory does not match trajectory_id: " +
            run_directory.filename().string());
  }

  const auto report_path = run_directory / "compile_report.yaml";
  const auto lock_path = canonical_runs_root.parent_path() / "curobo.lock";
  const auto robot_model_path = environment_directory / "robot/robot_model.yaml";
  const auto report_document = loadHashedYamlDocument(report_path, "compiled run report");
  const auto lock_document = loadHashedYamlDocument(lock_path, "workspace curobo.lock");
  const auto robot_model_document = loadHashedYamlDocument(
    robot_model_path, "compiled environment robot_model.yaml");
  if (sha256File(report_path) != report_document.sha256 ||
    sha256File(lock_path) != lock_document.sha256 ||
    sha256File(robot_model_path) != robot_model_document.sha256)
  {
    throw std::runtime_error(
            "compile_report.yaml, curobo.lock, or robot_model.yaml changed while being loaded");
  }
  const YAML::Node & report = report_document.root;
  const YAML::Node & lock = lock_document.root;
  const YAML::Node & robot_model = robot_model_document.root;
  if (!report.IsMap() || !lock.IsMap() || !robot_model.IsMap()) {
    throw std::runtime_error(
            "compile_report.yaml, curobo.lock, and robot_model.yaml roots must be maps");
  }
  CompiledRunEvidence evidence;
  try {
    if (lock["schema_version"].as<int>() != 1) {
      throw std::runtime_error("curobo.lock schema_version must be 1");
    }
    if (requiredString(report, "environment_id") != expected_environment_id) {
      throw std::runtime_error("compile_report.yaml environment_id does not match action goal");
    }
    if (requiredString(report, "trajectory_id") != expected_trajectory_id) {
      throw std::runtime_error("compile_report.yaml trajectory_id does not match action goal");
    }
    evidence = parseCompileReportEvidence(report, robot_model);
    evidence.compile_report_sha256 = report_document.sha256;
    const std::string locked_revision = normalizedGitCommit(requiredString(lock, "commit"));
    if (evidence.curobo_revision != locked_revision) {
      throw std::runtime_error(
              "compile_report.yaml curobo_revision does not match workspace curobo.lock");
    }
    const auto reported_robot_model = evidence.environment_hashes.find(
      "robot/robot_model.yaml");
    if (reported_robot_model == evidence.environment_hashes.end() ||
      reported_robot_model->second != robot_model_document.sha256)
    {
      throw std::runtime_error(
              "compile_report.yaml robot/robot_model.yaml hash does not match parsed bytes");
    }
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "compile_report.yaml or curobo.lock has invalid fields: " +
            std::string(error.what()));
  }
  validateCompileEvidenceMetrics(evidence);
  if (source_contents.sha256 != evidence.source_trajectory_sha256) {
    throw std::runtime_error(
            "compile_report.yaml source_trajectory_sha256 does not match the compiled run source");
  }
  if (joint_contents.sha256 != evidence.joint_trajectory_sha256) {
    throw std::runtime_error(
            "compile_report.yaml joint_trajectory_sha256 does not match the compiled run trajectory");
  }
  const auto robot_validation_model = loadRobotValidationModel(environment_directory);
  auto canonical_parsed = parseJointTrajectory(
    joint_contents.bytes, robot_validation_model.joint_names);
  const auto canonical_dynamics = validateCanonicalTrajectoryDynamics(
    canonical_parsed.first, robot_validation_model);
  const auto kinematics_context = SimulationRecordingValidator::loadKinematicsContext(
    environment_directory, canonical_source, robot_validation_model.joint_names);
  const auto canonical_tip = SimulationRecordingValidator::validateCanonicalSourcePoses(
    canonical_parsed.first, kinematics_context);
  requireRecomputedCanonicalEvidenceMatchesReport(
    canonical_dynamics, canonical_tip, evidence);
  const auto source_timing = parseSourceTimingEvidence(source_contents.bytes);
  const auto joint_timing = parseJointTimingEvidence(joint_contents.bytes);
  const std::size_t matched_source_count = verifyAuthoritativeTiming(
    source_timing, joint_timing);
  if (source_timing.time_ns.size() != evidence.source_waypoint_count ||
    matched_source_count != evidence.matched_source_waypoint_count ||
    source_timing.time_ns.back() !=
    nanosecondsFromDuration(durationFromSeconds(evidence.source_duration_s)) ||
    joint_timing.time_ns.size() != evidence.point_count ||
    joint_timing.time_ns.back() != nanosecondsFromDuration(durationFromSeconds(evidence.duration_s)))
  {
    throw std::runtime_error(
            "compile_report.yaml authoritative timing evidence does not match source/joint CSV");
  }
  if (source_contents.sha256 != evidence.source_trajectory_sha256) {
    throw std::runtime_error(
            "compile_report.yaml source_trajectory_sha256 does not match the compiled run source");
  }
  if (joint_contents.sha256 != evidence.joint_trajectory_sha256) {
    throw std::runtime_error(
            "compile_report.yaml joint_trajectory_sha256 does not match the compiled run trajectory");
  }
  if (sha256File(report_path) != evidence.compile_report_sha256) {
    throw std::runtime_error("compile_report.yaml changed during compiled-run validation");
  }
  if (sha256File(lock_path) != lock_document.sha256 ||
    sha256File(robot_model_path) != robot_model_document.sha256 ||
    sha256File(canonical_source) != source_contents.sha256 ||
    sha256File(canonical_trajectory) != joint_contents.sha256)
  {
    throw std::runtime_error(
            "compiled-run lock, robot model, source, or joint trajectory changed during validation");
  }
  const auto authoritative_source = std::filesystem::canonical(
    environment_directory / "trajectories" / (expected_trajectory_id + ".csv"));
  const auto authoritative_source_contents = loadHashedFileContents(
    authoritative_source, "authoritative environment source trajectory");
  if (authoritative_source_contents.sha256 != evidence.source_trajectory_sha256) {
    throw std::runtime_error(
            "compile_report.yaml source hash does not match the current environment trajectory");
  }

  const auto current_hashes = currentEnvironmentHashes(environment_directory);
  if (evidence.environment_hashes != current_hashes) {
    throw std::runtime_error(
            "compile_report.yaml environment_hashes does not exactly match the current environment file set");
  }
  const auto planner_path = environment_directory / "planner.yaml";
  const auto planner_document = loadHashedYamlDocument(
    planner_path, "compiled environment planner.yaml");
  const auto planner_hash = evidence.environment_hashes.find("planner.yaml");
  if (planner_hash == evidence.environment_hashes.end() ||
    planner_document.sha256 != planner_hash->second)
  {
    throw std::runtime_error(
            "compile_report.yaml planner.yaml hash does not match parsed bytes");
  }
  verifyUniformTimeScalePolicy(planner_document.root, evidence.uniform_time_scale);
  verifyCompileValidationPolicy(planner_document.root, evidence);
  if (sha256File(planner_path) != planner_document.sha256) {
    throw std::runtime_error("planner.yaml changed during compiled-run validation");
  }
  if (sha256File(authoritative_source) != authoritative_source_contents.sha256) {
    throw std::runtime_error(
            "authoritative environment source trajectory changed during compiled-run validation");
  }

  evidence.run_directory = run_directory;
  evidence.source_trajectory_path = canonical_source;
  evidence.joint_trajectory_path = canonical_trajectory;
  evidence.compile_report_path = std::filesystem::canonical(report_path);
  return evidence;
}

trajectory_msgs::msg::JointTrajectory RecordingStore::loadJointTrajectoryCsv(
  const std::filesystem::path & trajectory_path,
  const std::vector<std::string> & joint_names,
  std::vector<std::string> * phases)
{
  const auto contents = loadHashedFileContents(
    trajectory_path, "compiled joint trajectory");
  auto parsed = parseJointTrajectory(contents.bytes, joint_names);
  if (sha256File(trajectory_path) != contents.sha256) {
    throw std::runtime_error("compiled joint trajectory changed while it was being parsed");
  }
  if (phases != nullptr) {
    *phases = std::move(parsed.second);
  }
  return std::move(parsed.first);
}

std::string RecordingStore::sha256File(const std::filesystem::path & path)
{
  requireRegularFile(path, "SHA-256 input");
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to open file for SHA-256: " + path.string());
  }
  Sha256 hash;
  std::array<char, 1024U * 1024U> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = input.gcount();
    if (count > 0) {
      hash.update(
        reinterpret_cast<const std::uint8_t *>(buffer.data()), static_cast<std::size_t>(count));
    }
  }
  if (!input.eof()) {
    throw std::runtime_error("failed while reading file for SHA-256: " + path.string());
  }
  const auto digest = hash.finish();
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const std::uint8_t byte : digest) {
    output << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return output.str();
}

const std::filesystem::path & RecordingStore::environmentsRoot() const noexcept
{
  return environments_root_;
}

}  // namespace hx_waypoint
