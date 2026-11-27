// Copyright 2026 Saif Sidhik
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <EAIK.h>

#if __has_include(<moveit/kinematics_base/kinematics_base.hpp>)
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/robot_model/joint_model.hpp>
#else
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/joint_model.h>
#endif

#include <eaik_kinematics_plugin/eaik_kinematics_parameters.hpp>

namespace eaik_kinematics_plugin
{
/// @brief MoveIt kinematics plugin that solves inverse kinematics in closed form with EAIK.
///
/// A group with fewer than 6 joints only gets solutions for poses that it can reach exactly.
///
/// Thread safety: initialize() writes all members. The const methods only read them, and
/// EAIK::Robot keeps no mutable state, so concurrent calls to the const methods are safe.
class EAIKKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  /// @brief Builds the EAIK model of the group from the robot model at its zero configuration.
  /// @param node Node that holds the robot_description_kinematics.<group> parameters.
  /// @param robot_model Robot model that contains the group.
  /// @param group_name Name of the joint model group to solve for.
  /// @param base_frame Frame in which IK poses are given.
  /// @param tip_frames Tip frames; exactly one is supported.
  /// @param search_discretization Not used, because a group of up to 6 joints has no redundant joints.
  /// @return True if the group is a chain of 1 to 6 revolute joints that EAIK solves in closed form.
  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                  const std::string& group_name, const std::string& base_frame,
                  const std::vector<std::string>& tip_frames, double search_discretization) override;

  /// @brief Gets the solution nearest to the seed, without a solution callback.
  /// @param ik_pose Tip pose in the base frame.
  /// @param ik_seed_state Seed joint values.
  /// @param solution Output joint values.
  /// @param error_code Output error code.
  /// @param options Query options.
  /// @return True if a solution was found.
  bool
  getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /// @brief Gets all solutions within the joint limits, sorted by distance to the seed.
  /// @param ik_poses Tip poses in the base frame; exactly one is supported.
  /// @param ik_seed_state Seed joint values, used for sorting.
  /// @param solutions Output solutions.
  /// @param result Output result; kinematic_error tells why no solution was returned.
  /// @param options Query options.
  /// @return True if at least one solution was found.
  bool getPositionIK(const std::vector<geometry_msgs::msg::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                     std::vector<std::vector<double>>& solutions, kinematics::KinematicsResult& result,
                     const kinematics::KinematicsQueryOptions& options) const override;

  /// @brief Delegates to the general searchPositionIK overload.
  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /// @brief Delegates to the general searchPositionIK overload.
  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /// @brief Delegates to the general searchPositionIK overload.
  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /// @brief Solves once and gives the solutions to the callback in order of distance to the seed.
  /// @param ik_pose Tip pose in the base frame.
  /// @param ik_seed_state Seed joint values.
  /// @param timeout Not used; the solver is analytical and does not iterate.
  /// @param consistency_limits Empty, or the maximum distance from the seed for each joint.
  /// @param solution Output joint values: the first solution that the callback accepts.
  /// @param solution_callback Optional callback that accepts or rejects a solution.
  /// @param error_code Output error code; NO_IK_SOLUTION if no solution is accepted.
  /// @param options Query options.
  /// @return True if a solution was accepted.
  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      const IKCallbackFn& solution_callback, moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /// @brief Computes the tip pose in the base frame with the EAIK model.
  /// @param link_names Links to compute; only the tip frame is supported.
  /// @param joint_angles Joint values.
  /// @param poses Output poses in the base frame.
  /// @return True if all links are the tip frame and the joint vector has the correct size.
  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override;

  /// @brief Gets the active joint names in chain order.
  const std::vector<std::string>& getJointNames() const override;

  /// @brief Gets the link names for which FK is available: the tip frame only.
  const std::vector<std::string>& getLinkNames() const override;

  /// @brief Checks that the group is a chain of 1 to 6 revolute, non-mimic joints.
  /// @param jmg Group to check.
  /// @param error_text_out Optional output for the reason if the group is not supported.
  /// @return True if the group is supported.
  bool supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error_text_out = nullptr) const override;

private:
  /// @brief Gets all EAIK solutions that reach the pose within tolerance and obey the joint and consistency
  /// limits, including the +/-2 pi variants, sorted by distance to the seed.
  /// @param ik_pose Tip pose in the base frame.
  /// @param ik_seed_state Seed joint values.
  /// @param consistency_limits Empty, or the maximum distance from the seed for each joint.
  /// @return The solutions; empty if there are none.
  std::vector<std::vector<double>> computeSolutions(const geometry_msgs::msg::Pose& ik_pose,
                                                    const std::vector<double>& ik_seed_state,
                                                    const std::vector<double>& consistency_limits) const;

  /// @brief Checks that EAIK reproduces sampled configurations from the poses of its own forward kinematics.
  ///
  /// EAIK accepts some groups that it then cannot solve, for example an arm with rounded URDF angles.
  /// @return True if each sampled configuration is the solution nearest to itself.
  bool solvesOwnForwardKinematics() const;

  /// @brief Adds every value of a joint that is equal modulo 2 pi and within its limits.
  /// @param joint Joint model that holds the limits.
  /// @param value Joint value from EAIK.
  /// @return The values; empty if no value is within the limits.
  static std::vector<double> getValuesWithinLimits(const moveit::core::JointModel& joint, double value);

  std::unique_ptr<EAIK::Robot> robot_;
  std::vector<const moveit::core::JointModel*> joints_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  eaik_kinematics::Params params_;
};
}  // namespace eaik_kinematics_plugin
