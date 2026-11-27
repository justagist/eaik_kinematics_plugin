// Copyright 2026 Saif Sidhik
// SPDX-License-Identifier: BSD-3-Clause

#include "eaik_kinematics_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#if __has_include(<moveit/robot_model/robot_model.hpp>)
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#else
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#endif

#include <pluginlib/class_list_macros.hpp>

namespace eaik_kinematics_plugin
{
namespace
{
/// Margin by which an EAIK joint value may exceed a limit before it is clamped to the limit.
constexpr double kLimitMargin = 1e-9;
constexpr std::size_t kMaxJoints = 6;

rclcpp::Logger getLogger()
{
  return rclcpp::get_logger("eaik_kinematics_plugin");
}

/// @brief Converts a pose message to a homogeneous transform.
/// @param pose Pose to convert.
/// @return The transform.
IKS::Homogeneous_T toTransform(const geometry_msgs::msg::Pose& pose)
{
  const Eigen::Quaterniond orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  IKS::Homogeneous_T transform = IKS::Homogeneous_T::Identity();
  transform.topLeftCorner<3, 3>() = orientation.normalized().toRotationMatrix();
  transform.topRightCorner<3, 1>() << pose.position.x, pose.position.y, pose.position.z;
  return transform;
}

/// @brief Converts a homogeneous transform to a pose message.
/// @param transform Transform to convert.
/// @return The pose.
geometry_msgs::msg::Pose toPose(const IKS::Homogeneous_T& transform)
{
  const Eigen::Quaterniond orientation(Eigen::Matrix3d(transform.topLeftCorner<3, 3>()));
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform(0, 3);
  pose.position.y = transform(1, 3);
  pose.position.z = transform(2, 3);
  pose.orientation.w = orientation.w();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  return pose;
}
}  // namespace

bool EAIKKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                                      const std::string& group_name, const std::string& base_frame,
                                      const std::vector<std::string>& tip_frames, double search_discretization)
{
  if (tip_frames.size() != 1)
  {
    RCLCPP_ERROR(getLogger(), "Group '%s': EAIK supports exactly one tip frame, but %zu were given", group_name.c_str(),
                 tip_frames.size());
    return false;
  }

  params_ = eaik_kinematics::ParamListener(node, "robot_description_kinematics." + group_name).get_params();
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);
  if (!jmg)
  {
    return false;
  }

  std::string error;
  if (!supportsGroup(jmg, &error))
  {
    RCLCPP_ERROR(getLogger(), "Group '%s' is not supported by EAIK: %s", group_name.c_str(), error.c_str());
    return false;
  }

  const moveit::core::LinkModel* base_link = robot_model_->getLinkModel(base_frame_);
  const moveit::core::LinkModel* tip_link = robot_model_->getLinkModel(getTipFrame());
  if (!base_link || !tip_link)
  {
    RCLCPP_ERROR(getLogger(), "Group '%s': base frame '%s' or tip frame '%s' is not a link of the robot",
                 group_name.c_str(), base_frame_.c_str(), getTipFrame().c_str());
    return false;
  }

  joints_ = jmg->getActiveJointModels();
  // The EAIK model starts at the base frame and ends at the tip frame, so no movable joint outside the
  // group may lie between them and the chain.
  if (moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(base_link) !=
      moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(joints_.front()->getParentLinkModel()))
  {
    RCLCPP_ERROR(getLogger(), "Group '%s': base frame '%s' is not rigidly connected to the parent link of joint '%s'",
                 group_name.c_str(), base_frame_.c_str(), joints_.front()->getName().c_str());
    return false;
  }
  if (moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(tip_link) != joints_.back()->getChildLinkModel())
  {
    RCLCPP_ERROR(getLogger(), "Group '%s': tip frame '%s' is not rigidly connected to the child link of joint '%s'",
                 group_name.c_str(), getTipFrame().c_str(), joints_.back()->getName().c_str());
    return false;
  }

  // EAIK takes the joint axes (H) and the offsets between consecutive joint origins (P) in the base frame
  // at the zero configuration, plus the fixed tip rotation at that configuration (R6T).
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(jmg, std::vector<double>(joints_.size(), 0.0));
  state.updateLinkTransforms();
  const Eigen::Isometry3d base_inverse = state.getGlobalLinkTransform(base_link).inverse();

  Eigen::MatrixXd h(3, joints_.size());
  Eigen::MatrixXd p(3, joints_.size() + 1);
  Eigen::Vector3d previous_origin = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    // At zero, the child link frame of a revolute joint is its joint frame, in which its axis is given.
    const Eigen::Isometry3d joint_frame = base_inverse * state.getGlobalLinkTransform(joints_[i]->getChildLinkModel());
    const auto* revolute = static_cast<const moveit::core::RevoluteJointModel*>(joints_[i]);
    h.col(i) = joint_frame.linear() * revolute->getAxis();
    p.col(i) = joint_frame.translation() - previous_origin;
    previous_origin = joint_frame.translation();
  }
  const Eigen::Isometry3d tip_frame = base_inverse * state.getGlobalLinkTransform(tip_link);
  p.col(joints_.size()) = tip_frame.translation() - previous_origin;
  const Eigen::Matrix3d tip_rotation = tip_frame.linear();

  try
  {
    robot_ = std::make_unique<EAIK::Robot>(h, p, tip_rotation);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(getLogger(), "Group '%s': EAIK cannot model the group: %s", group_name.c_str(), e.what());
    return false;
  }
  if (!robot_->has_known_decomposition())
  {
    RCLCPP_ERROR(getLogger(),
                 "Group '%s': EAIK knows no closed-form decomposition for this kinematic structure. "
                 "Use a different kinematics solver for this group.",
                 group_name.c_str());
    robot_.reset();
    return false;
  }

  joint_names_ = jmg->getActiveJointModelNames();
  link_names_ = { getTipFrame() };
  RCLCPP_INFO(getLogger(), "Group '%s': EAIK kinematic family '%s'", group_name.c_str(),
              robot_->get_kinematic_family().c_str());
  return true;
}

bool EAIKKinematicsPlugin::supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error_text_out) const
{
  const auto fail = [error_text_out](const std::string& error) {
    if (error_text_out)
    {
      *error_text_out = error;
    }
    return false;
  };

  if (!jmg->isChain())
  {
    return fail("the group is not a chain");
  }
  if (!jmg->getMimicJointModels().empty())
  {
    return fail("the group has mimic joints");
  }
  for (const moveit::core::JointModel* joint : jmg->getActiveJointModels())
  {
    if (joint->getType() != moveit::core::JointModel::REVOLUTE)
    {
      return fail("joint '" + joint->getName() + "' is not revolute");
    }
  }
  const std::size_t num_joints = jmg->getActiveJointModels().size();
  if (num_joints == 0 || num_joints > kMaxJoints)
  {
    return fail("the group has " + std::to_string(num_joints) + " joints, but EAIK supports 1 to 6 joints only");
  }
  return true;
}

bool EAIKKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                         const std::vector<double>& ik_seed_state, std::vector<double>& solution,
                                         moveit_msgs::msg::MoveItErrorCodes& error_code,
                                         const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, 0.0, {}, solution, IKCallbackFn(), error_code, options);
}

bool EAIKKinematicsPlugin::getPositionIK(const std::vector<geometry_msgs::msg::Pose>& ik_poses,
                                         const std::vector<double>& ik_seed_state,
                                         std::vector<std::vector<double>>& solutions,
                                         kinematics::KinematicsResult& result,
                                         const kinematics::KinematicsQueryOptions& /*options*/) const
{
  solutions.clear();
  result.solution_percentage = 0.0;
  if (!robot_)
  {
    result.kinematic_error = kinematics::KinematicErrors::SOLVER_NOT_ACTIVE;
    return false;
  }
  if (ik_poses.empty())
  {
    result.kinematic_error = kinematics::KinematicErrors::EMPTY_TIP_POSES;
    return false;
  }
  if (ik_poses.size() > 1)
  {
    result.kinematic_error = kinematics::KinematicErrors::MULTIPLE_TIPS_NOT_SUPPORTED;
    return false;
  }
  if (ik_seed_state.size() != joints_.size())
  {
    RCLCPP_ERROR(getLogger(), "Seed state must have size %zu instead of %zu", joints_.size(), ik_seed_state.size());
    result.kinematic_error = kinematics::KinematicErrors::NO_SOLUTION;
    return false;
  }

  solutions = computeSolutions(ik_poses.front(), ik_seed_state, {});
  if (solutions.empty())
  {
    result.kinematic_error = kinematics::KinematicErrors::NO_SOLUTION;
    return false;
  }
  result.kinematic_error = kinematics::KinematicErrors::OK;
  result.solution_percentage = 1.0;
  return true;
}

bool EAIKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            std::vector<double>& solution,
                                            moveit_msgs::msg::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, {}, solution, IKCallbackFn(), error_code, options);
}

bool EAIKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            const std::vector<double>& consistency_limits,
                                            std::vector<double>& solution,
                                            moveit_msgs::msg::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool EAIKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                            moveit_msgs::msg::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, {}, solution, solution_callback, error_code, options);
}

bool EAIKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double /*timeout*/,
                                            const std::vector<double>& consistency_limits,
                                            std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                            moveit_msgs::msg::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& /*options*/) const
{
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  if (!robot_)
  {
    RCLCPP_ERROR(getLogger(), "EAIK kinematics solver is not initialised");
    return false;
  }
  if (ik_seed_state.size() != joints_.size())
  {
    RCLCPP_ERROR(getLogger(), "Seed state must have size %zu instead of %zu", joints_.size(), ik_seed_state.size());
    return false;
  }
  if (!consistency_limits.empty() && consistency_limits.size() != joints_.size())
  {
    RCLCPP_ERROR(getLogger(), "Consistency limits must be empty or have size %zu instead of %zu", joints_.size(),
                 consistency_limits.size());
    return false;
  }

  for (const std::vector<double>& candidate : computeSolutions(ik_pose, ik_seed_state, consistency_limits))
  {
    if (solution_callback)
    {
      solution_callback(ik_pose, candidate, error_code);
      if (error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      {
        continue;
      }
    }
    solution = candidate;
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;
  }
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  return false;
}

std::vector<std::vector<double>>
EAIKKinematicsPlugin::computeSolutions(const geometry_msgs::msg::Pose& ik_pose,
                                       const std::vector<double>& ik_seed_state,
                                       const std::vector<double>& consistency_limits) const
{
  const IKS::Homogeneous_T target = toTransform(ik_pose);
  IKS::IK_Solution eaik_solutions;
  try
  {
    eaik_solutions = robot_->calculate_IK(target);
  }
  catch (const std::exception& e)
  {
    RCLCPP_DEBUG(getLogger(), "EAIK failed to solve: %s", e.what());
    return {};
  }

  std::vector<std::pair<double, std::vector<double>>> sorted;
  for (const std::vector<double>& q : eaik_solutions.Q)
  {
    if (!std::all_of(q.begin(), q.end(), [](double value) { return std::isfinite(value); }))
    {
      continue;
    }

    // EAIK also returns least-squares solutions when the pose is out of reach, so only solutions whose forward
    // kinematics reach the target are kept.
    const IKS::Homogeneous_T reached = robot_->fwdkin(q);
    const double position_error = (reached.topRightCorner<3, 1>() - target.topRightCorner<3, 1>()).norm();
    const Eigen::AngleAxisd rotation_error(
        Eigen::Matrix3d(reached.topLeftCorner<3, 3>().transpose() * target.topLeftCorner<3, 3>()));
    if (position_error > params_.position_tolerance || rotation_error.angle() > params_.orientation_tolerance)
    {
      continue;
    }

    std::vector<std::vector<double>> combinations(1);
    for (std::size_t i = 0; i < joints_.size() && !combinations.empty(); ++i)
    {
      std::vector<double> values = getValuesWithinLimits(*joints_[i], q[i]);
      if (!consistency_limits.empty())
      {
        values.erase(std::remove_if(values.begin(), values.end(),
                                    [&](double value) {
                                      return joints_[i]->distance(&value, &ik_seed_state[i]) > consistency_limits[i];
                                    }),
                     values.end());
      }

      std::vector<std::vector<double>> extended;
      for (const std::vector<double>& prefix : combinations)
      {
        for (double value : values)
        {
          extended.push_back(prefix);
          extended.back().push_back(value);
        }
      }
      combinations = std::move(extended);
    }

    for (std::vector<double>& combination : combinations)
    {
      double distance = 0.0;
      for (std::size_t i = 0; i < joints_.size(); ++i)
      {
        distance += joints_[i]->distance(&combination[i], &ik_seed_state[i]);
      }
      sorted.emplace_back(distance, std::move(combination));
    }
  }

  std::sort(sorted.begin(), sorted.end(), [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
  std::vector<std::vector<double>> solutions;
  solutions.reserve(sorted.size());
  for (auto& [distance, solution] : sorted)
  {
    solutions.push_back(std::move(solution));
  }
  return solutions;
}

std::vector<double> EAIKKinematicsPlugin::getValuesWithinLimits(const moveit::core::JointModel& joint, double value)
{
  const moveit::core::VariableBounds& bounds = joint.getVariableBounds().front();
  if (!bounds.position_bounded_)
  {
    // A continuous joint wraps its value into [-pi, pi].
    joint.enforcePositionBounds(&value);
    return { value };
  }

  std::vector<double> values;
  const double lowest = bounds.min_position_ - kLimitMargin;
  double candidate = value + 2.0 * M_PI * std::ceil((lowest - value) / (2.0 * M_PI));
  for (; candidate <= bounds.max_position_ + kLimitMargin; candidate += 2.0 * M_PI)
  {
    values.push_back(std::clamp(candidate, bounds.min_position_, bounds.max_position_));
  }
  return values;
}

bool EAIKKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                         const std::vector<double>& joint_angles,
                                         std::vector<geometry_msgs::msg::Pose>& poses) const
{
  if (!robot_)
  {
    RCLCPP_ERROR(getLogger(), "EAIK kinematics solver is not initialised");
    return false;
  }
  if (joint_angles.size() != joints_.size())
  {
    RCLCPP_ERROR(getLogger(), "Joint angles must have size %zu instead of %zu", joints_.size(), joint_angles.size());
    return false;
  }
  for (const std::string& link_name : link_names)
  {
    if (link_name != getTipFrame())
    {
      RCLCPP_ERROR(getLogger(), "EAIK computes forward kinematics for the tip frame '%s' only, not for '%s'",
                   getTipFrame().c_str(), link_name.c_str());
      return false;
    }
  }

  poses.assign(link_names.size(), toPose(robot_->fwdkin(joint_angles)));
  return true;
}

const std::vector<std::string>& EAIKKinematicsPlugin::getJointNames() const
{
  return joint_names_;
}

const std::vector<std::string>& EAIKKinematicsPlugin::getLinkNames() const
{
  return link_names_;
}
}  // namespace eaik_kinematics_plugin

PLUGINLIB_EXPORT_CLASS(eaik_kinematics_plugin::EAIKKinematicsPlugin, kinematics::KinematicsBase)
