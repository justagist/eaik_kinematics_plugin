// Copyright 2026 Saif Sidhik
// SPDX-License-Identifier: BSD-3-Clause

// Compares the EAIK and KDL kinematics plugins on the Fanuc arm from moveit_resources and prints a
// Markdown table. Usage: benchmark_eaik_kinematics [num_poses]

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#if __has_include(<moveit/kinematics_base/kinematics_base.hpp>)
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/utils/robot_model_test_utils.hpp>
#else
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>
#endif

namespace
{
constexpr char kGroup[] = "manipulator";
constexpr char kBase[] = "base_link";
constexpr char kTip[] = "tool0";
// The kinematics_solver_timeout of the Fanuc MoveIt config.
constexpr double kTimeout = 0.005;

struct Result
{
  std::string name;
  double reachable_success = 0.0;
  double reachable_median_us = 0.0;
  double reachable_p95_us = 0.0;
  double unreachable_median_us = 0.0;
  double unreachable_p95_us = 0.0;
  std::string solutions_per_pose = "n/a";
};

/// @brief Gets the value at a fraction of the sorted samples.
/// @param samples Samples; sorted in place.
/// @param fraction Fraction in [0, 1].
/// @return The value.
double getPercentile(std::vector<double>& samples, double fraction)
{
  std::sort(samples.begin(), samples.end());
  return samples[static_cast<std::size_t>(fraction * static_cast<double>(samples.size() - 1))];
}

/// @brief Times one searchPositionIK call.
/// @param solver Solver to call.
/// @param pose Target pose.
/// @param seed Seed joint values.
/// @param solved Output: true if the call returned a solution.
/// @return Duration of the call in microseconds.
double timeSearch(const kinematics::KinematicsBase& solver, const geometry_msgs::msg::Pose& pose,
                  const std::vector<double>& seed, bool& solved)
{
  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  const auto start = std::chrono::steady_clock::now();
  solved = solver.searchPositionIK(pose, seed, kTimeout, solution, error_code);
  return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const int num_poses = argc > 1 ? std::stoi(argv[1]) : 1000;

  const auto node = std::make_shared<rclcpp::Node>("benchmark_eaik_kinematics");
  const moveit::core::RobotModelPtr robot_model = moveit::core::loadTestingRobotModel("fanuc");
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(kGroup);

  std::mt19937 rng(42);
  const auto get_random_configuration = [&]() {
    std::vector<double> q;
    for (const moveit::core::JointModel* joint : jmg->getActiveJointModels())
    {
      const moveit::core::VariableBounds& bounds = joint->getVariableBounds().front();
      q.push_back(std::uniform_real_distribution<double>(bounds.min_position_, bounds.max_position_)(rng));
    }
    return q;
  };

  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();
  std::vector<geometry_msgs::msg::Pose> reachable;
  std::vector<geometry_msgs::msg::Pose> unreachable;
  std::vector<std::vector<double>> seeds;
  for (int i = 0; i < num_poses; ++i)
  {
    state.setJointGroupPositions(jmg, get_random_configuration());
    state.updateLinkTransforms();
    Eigen::Isometry3d tip = state.getGlobalLinkTransform(kBase).inverse() * state.getGlobalLinkTransform(kTip);
    reachable.push_back(tf2::toMsg(tip));
    tip.translation().x() += 10.0;
    unreachable.push_back(tf2::toMsg(tip));
    seeds.push_back(get_random_configuration());
  }

  pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");
  std::vector<Result> results;
  for (const std::string plugin :
       { "eaik_kinematics_plugin/EAIKKinematicsPlugin", "kdl_kinematics_plugin/KDLKinematicsPlugin" })
  {
    const kinematics::KinematicsBasePtr solver = loader.createSharedInstance(plugin);
    if (!solver->initialize(node, *robot_model, kGroup, kBase, { kTip }, 0.1))
    {
      std::fprintf(stderr, "Failed to initialise %s\n", plugin.c_str());
      return 1;
    }

    Result result;
    result.name = plugin;
    std::vector<double> reachable_us;
    std::vector<double> unreachable_us;
    int successes = 0;
    for (int i = 0; i < num_poses; ++i)
    {
      bool solved = false;
      reachable_us.push_back(timeSearch(*solver, reachable[i], seeds[i], solved));
      successes += solved ? 1 : 0;
      unreachable_us.push_back(timeSearch(*solver, unreachable[i], seeds[i], solved));
    }
    result.reachable_success = 100.0 * successes / num_poses;
    result.reachable_median_us = getPercentile(reachable_us, 0.5);
    result.reachable_p95_us = getPercentile(reachable_us, 0.95);
    result.unreachable_median_us = getPercentile(unreachable_us, 0.5);
    result.unreachable_p95_us = getPercentile(unreachable_us, 0.95);

    std::vector<double> counts;
    for (int i = 0; i < num_poses; ++i)
    {
      std::vector<std::vector<double>> solutions;
      kinematics::KinematicsResult kinematics_result;
      if (solver->getPositionIK({ reachable[i] }, seeds[i], solutions, kinematics_result,
                                kinematics::KinematicsQueryOptions()))
        counts.push_back(static_cast<double>(solutions.size()));
    }
    if (!counts.empty())
    {
      const double median = getPercentile(counts, 0.5);
      char text[64];
      std::snprintf(text, sizeof(text), "%.0f (min %.0f, max %.0f)", median, counts.front(), counts.back());
      result.solutions_per_pose = text;
    }
    results.push_back(result);
  }

  std::printf("Fanuc M-10iA `%s`, %d poses, timeout %.3f s\n\n", kGroup, num_poses, kTimeout);
  std::printf("| Plugin | Reachable: success | Reachable: median / p95 (us) | Unreachable: median / p95 (us) | "
              "getPositionIKs: median solutions |\n");
  std::printf("|---|---|---|---|---|\n");
  for (const Result& result : results)
  {
    std::printf("| %s | %.1f %% | %.1f / %.1f | %.1f / %.1f | %s |\n", result.name.c_str(), result.reachable_success,
                result.reachable_median_us, result.reachable_p95_us, result.unreachable_median_us,
                result.unreachable_p95_us, result.solutions_per_pose.c_str());
  }

  rclcpp::shutdown();
  // MoveIt's logger keeps a static node. Its destructor at exit calls rmw_zenoh after zenoh has shut down,
  // which aborts the process. Skip the static destructors.
  std::fflush(stdout);
  std::quick_exit(0);
}
