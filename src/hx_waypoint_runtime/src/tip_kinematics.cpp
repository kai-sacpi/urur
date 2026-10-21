#include "hx_waypoint/tip_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace hx_waypoint
{
namespace
{

template<std::size_t Size>
void requireFinite(const std::array<double, Size> & values, const std::string & field)
{
  if (!std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);})) {
    throw std::invalid_argument(field + " contains a non-finite value");
  }
}

void requireFinite(const std::vector<double> & values, const std::string & field)
{
  if (!std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);})) {
    throw std::invalid_argument(field + " contains a non-finite value");
  }
}

}  // namespace

struct TipKinematics::Impl
{
  Impl(
    const std::filesystem::path & urdf_path,
    const std::string & base_link,
    const std::string & tool_link,
    const std::string & tip_frame,
    const std::array<double, 3> & translation,
    const std::array<double, 4> & quaternion,
    std::vector<std::string> expected_joint_names,
    std::vector<std::string> expected_frame_names)
  : joint_names(std::move(expected_joint_names))
  {
    if (!std::filesystem::is_regular_file(urdf_path)) {
      throw std::invalid_argument("tip kinematics URDF does not exist: " + urdf_path.string());
    }
    if (base_link.empty() || tool_link.empty() || tip_frame.empty() || joint_names.empty()) {
      throw std::invalid_argument("tip kinematics frame and joint names must not be empty");
    }
    requireFinite(translation, "tool-to-tip translation");
    requireFinite(quaternion, "tool-to-tip quaternion");
    const double quaternion_norm = std::sqrt(
      quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
      quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
    if (std::abs(quaternion_norm - 1.0) > 1.0e-3) {
      throw std::invalid_argument("tool-to-tip quaternion is not normalized");
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(urdf_path.string(), tree)) {
      throw std::runtime_error("failed to parse tip kinematics URDF: " + urdf_path.string());
    }
    if (!tree.getChain(base_link, tool_link, chain)) {
      throw std::runtime_error(
              "URDF has no kinematic chain from " + base_link + " to " + tool_link);
    }
    const KDL::Frame tool_to_tip(
      KDL::Rotation::Quaternion(
        quaternion[0] / quaternion_norm,
        quaternion[1] / quaternion_norm,
        quaternion[2] / quaternion_norm,
        quaternion[3] / quaternion_norm),
      KDL::Vector(translation[0], translation[1], translation[2]));
    chain.addSegment(
      KDL::Segment(tip_frame, KDL::Joint(tip_frame + "_fixed", KDL::Joint::Fixed), tool_to_tip));

    std::vector<std::string> chain_joint_names;
    chain_joint_names.reserve(chain.getNrOfJoints());
    for (const auto & segment : chain.segments) {
      if (segment.getJoint().getType() != KDL::Joint::Fixed) {
        chain_joint_names.push_back(segment.getJoint().getName());
      }
    }
    if (chain_joint_names != joint_names) {
      throw std::runtime_error("URDF chain joint order does not match the recording joint_names");
    }

    fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
    jacobian_solver = std::make_unique<KDL::ChainJntToJacSolver>(chain);

    if (std::unordered_set<std::string>(
        expected_frame_names.begin(), expected_frame_names.end()).size() !=
      expected_frame_names.size())
    {
      throw std::invalid_argument("tip kinematics frame_names must not contain duplicates");
    }
    const std::unordered_map<std::string, std::size_t> joint_indices = [&]() {
        std::unordered_map<std::string, std::size_t> result;
        for (std::size_t index = 0U; index < joint_names.size(); ++index) {
          result.emplace(joint_names[index], index);
        }
        return result;
      }();
    frame_chains.reserve(expected_frame_names.size());
    for (const auto & frame_name : expected_frame_names) {
      if (frame_name.empty()) {
        throw std::invalid_argument("tip kinematics frame_names must not contain empty values");
      }
      FrameChain frame_chain;
      frame_chain.frame_id = frame_name;
      if (frame_name == tip_frame) {
        frame_chain.chain = chain;
      } else if (!tree.getChain(base_link, frame_name, frame_chain.chain)) {
        throw std::runtime_error(
                "URDF has no kinematic chain from " + base_link + " to " + frame_name);
      }
      for (const auto & segment : frame_chain.chain.segments) {
        if (segment.getJoint().getType() == KDL::Joint::Fixed) {
          continue;
        }
        const auto found = joint_indices.find(segment.getJoint().getName());
        if (found == joint_indices.end()) {
          throw std::runtime_error(
                  "frame chain contains a joint outside recording joint_names: " +
                  segment.getJoint().getName());
        }
        frame_chain.joint_indices.push_back(found->second);
      }
      frame_chains.push_back(std::move(frame_chain));
      // KDL's solver retains a reference to its chain. Construct it only
      // after the FrameChain has reached its final reserved-vector address.
      frame_chains.back().fk_solver =
        std::make_unique<KDL::ChainFkSolverPos_recursive>(frame_chains.back().chain);
    }
  }

  TipKinematicsResult evaluate(
    const std::vector<double> & positions,
    const std::vector<double> & velocities)
  {
    if (positions.size() != joint_names.size() || velocities.size() != joint_names.size()) {
      throw std::invalid_argument("tip kinematics q and qd widths do not match joint_names");
    }
    requireFinite(positions, "tip kinematics positions");
    requireFinite(velocities, "tip kinematics velocities");

    KDL::JntArray q(chain.getNrOfJoints());
    Eigen::VectorXd qd(chain.getNrOfJoints());
    for (std::size_t index = 0; index < positions.size(); ++index) {
      q(static_cast<unsigned int>(index)) = positions[index];
      qd(static_cast<Eigen::Index>(index)) = velocities[index];
    }

    KDL::Frame tip_pose;
    KDL::Jacobian jacobian(chain.getNrOfJoints());
    if (fk_solver->JntToCart(q, tip_pose) < 0) {
      throw std::runtime_error("KDL failed to compute the rod-tip pose");
    }
    if (jacobian_solver->JntToJac(q, jacobian) < 0) {
      throw std::runtime_error("KDL failed to compute the rod-tip Jacobian");
    }
    const Eigen::VectorXd twist = jacobian.data * qd;

    TipKinematicsResult result;
    result.position_m = {{tip_pose.p.x(), tip_pose.p.y(), tip_pose.p.z()}};
    tip_pose.M.GetQuaternion(
      result.quaternion_xyzw[0], result.quaternion_xyzw[1],
      result.quaternion_xyzw[2], result.quaternion_xyzw[3]);
    result.linear_velocity_m_s = {{twist[0], twist[1], twist[2]}};
    result.angular_velocity_rad_s = {{twist[3], twist[4], twist[5]}};
    requireFinite(result.position_m, "computed tip position");
    requireFinite(result.quaternion_xyzw, "computed tip quaternion");
    requireFinite(result.linear_velocity_m_s, "computed tip linear velocity");
    requireFinite(result.angular_velocity_rad_s, "computed tip angular velocity");
    return result;
  }

  std::vector<FrameKinematicsResult> evaluateFrames(const std::vector<double> & positions)
  {
    if (positions.size() != joint_names.size()) {
      throw std::invalid_argument("frame kinematics q width does not match joint_names");
    }
    requireFinite(positions, "frame kinematics positions");
    std::vector<FrameKinematicsResult> results;
    results.reserve(frame_chains.size());
    for (const auto & frame_chain : frame_chains) {
      KDL::JntArray q(frame_chain.chain.getNrOfJoints());
      if (frame_chain.joint_indices.size() != frame_chain.chain.getNrOfJoints()) {
        throw std::runtime_error("frame kinematics joint map is inconsistent");
      }
      for (std::size_t index = 0U; index < frame_chain.joint_indices.size(); ++index) {
        q(static_cast<unsigned int>(index)) = positions[frame_chain.joint_indices[index]];
      }
      KDL::Frame pose;
      if (frame_chain.fk_solver->JntToCart(q, pose) < 0) {
        throw std::runtime_error(
                "KDL failed to compute frame pose for " + frame_chain.frame_id);
      }
      FrameKinematicsResult result;
      result.frame_id = frame_chain.frame_id;
      result.position_m = {{pose.p.x(), pose.p.y(), pose.p.z()}};
      pose.M.GetQuaternion(
        result.quaternion_xyzw[0], result.quaternion_xyzw[1],
        result.quaternion_xyzw[2], result.quaternion_xyzw[3]);
      requireFinite(result.position_m, "computed frame position");
      requireFinite(result.quaternion_xyzw, "computed frame quaternion");
      results.push_back(std::move(result));
    }
    return results;
  }

  struct FrameChain
  {
    std::string frame_id;
    KDL::Chain chain;
    std::vector<std::size_t> joint_indices;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  };

  std::vector<std::string> joint_names;
  KDL::Chain chain;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
  std::vector<FrameChain> frame_chains;
};

TipKinematics::TipKinematics(
  const std::filesystem::path & urdf_path,
  std::string base_link,
  std::string tool_link,
  std::string tip_frame,
  const std::array<double, 3> & tool_to_tip_translation_m,
  const std::array<double, 4> & tool_to_tip_quaternion_xyzw,
  std::vector<std::string> joint_names,
  std::vector<std::string> frame_names)
: impl_(std::make_unique<Impl>(
      urdf_path, base_link, tool_link, tip_frame, tool_to_tip_translation_m,
      tool_to_tip_quaternion_xyzw, std::move(joint_names), std::move(frame_names)))
{
}

TipKinematics::~TipKinematics() = default;
TipKinematics::TipKinematics(TipKinematics &&) noexcept = default;
TipKinematics & TipKinematics::operator=(TipKinematics &&) noexcept = default;

TipKinematicsResult TipKinematics::evaluate(
  const std::vector<double> & positions,
  const std::vector<double> & velocities)
{
  return impl_->evaluate(positions, velocities);
}

std::vector<FrameKinematicsResult> TipKinematics::evaluateFrames(
  const std::vector<double> & positions)
{
  return impl_->evaluateFrames(positions);
}

}  // namespace hx_waypoint
