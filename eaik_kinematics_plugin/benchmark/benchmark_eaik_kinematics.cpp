// Copyright 2026 Saif Sidhik
// SPDX-License-Identifier: BSD-3-Clause

// Compares the EAIK kinematics plugin with other MoveIt kinematics plugins on the Fanuc arm from
// moveit_resources, and prints Markdown tables. Plugins that are not installed are skipped.
// Usage: benchmark_eaik_kinematics [num_poses]

#include <stdlib.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
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
constexpr char kBase[] = "base_link";
// The kinematics_solver_timeout of the Fanuc MoveIt config.
constexpr double kTimeout = 0.005;
constexpr double kNearSeedNoise = 0.05;
constexpr double kSameBranchTolerance = 0.1;
constexpr double kNearSingularRange = 1e-3;
constexpr double kUnreachableOffset = 10.0;
constexpr std::size_t kWristJointIndex = 4;

// The Fanuc 6-joint group, and a 5-joint group of the same arm without its last joint.
constexpr char kSrdf[] = R"(<?xml version="1.0"?>
<robot name="fanuc">
  <group name="manipulator">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>
  <group name="five_joints">
    <chain base_link="base_link" tip_link="link_5"/>
  </group>
</robot>)";

const std::vector<std::string> kPlugins{
  "eaik_kinematics_plugin/EAIKKinematicsPlugin",
  "kdl_kinematics_plugin/KDLKinematicsPlugin",
  "cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin",
  "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin",
  "pick_ik/PickIkPlugin",
};

/// IK queries on one group: the target poses, the configurations they come from, and the seeds.
struct Case
{
  std::string group;
  std::string tip;
  std::vector<std::vector<double>> targets;
  std::vector<geometry_msgs::msg::Pose> poses;
  std::vector<std::vector<double>> seeds;
};

/// Results of one plugin on one case. Errors and distances are over the returned solutions only.
struct Stats
{
  double success_percent = 0.0;
  double same_branch_percent = 0.0;
  double median_us = 0.0;
  double p95_us = 0.0;
  double median_seed_distance = 0.0;
  double max_position_error = 0.0;
  double max_orientation_error = 0.0;
};

/// @brief Gets the value at a fraction of the sorted samples.
/// @param samples Samples; sorted in place.
/// @param fraction Fraction in [0, 1].
/// @return The value, or NaN if there are no samples.
double getPercentile(std::vector<double>& samples, double fraction)
{
  if (samples.empty())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(samples.begin(), samples.end());
  return samples[static_cast<std::size_t>(fraction * static_cast<double>(samples.size() - 1))];
}

/// @brief Formats a value for a table cell.
/// @param format printf format for one double.
/// @param value Value; NaN is shown as "-".
/// @return The text.
std::string format(const char* format, double value)
{
  if (std::isnan(value))
  {
    return "-";
  }
  char text[64];
  std::snprintf(text, sizeof(text), format, value);
  return text;
}

/// @brief Draws joint values uniformly within the joint limits of a group.
/// @param jmg Group.
/// @param rng Random number generator.
/// @return The joint values.
std::vector<double> getRandomConfiguration(const moveit::core::JointModelGroup& jmg, std::mt19937& rng)
{
  std::vector<double> q;
  for (const moveit::core::JointModel* joint : jmg.getActiveJointModels())
  {
    const moveit::core::VariableBounds& bounds = joint->getVariableBounds().front();
    q.push_back(std::uniform_real_distribution<double>(bounds.min_position_, bounds.max_position_)(rng));
  }
  return q;
}

/// @brief Computes the tip pose in the base frame with MoveIt.
/// @param state Scratch state of the robot.
/// @param group Group name.
/// @param tip Tip link name.
/// @param q Joint values of the group.
/// @return The tip pose.
Eigen::Isometry3d getTipTransform(moveit::core::RobotState& state, const std::string& group, const std::string& tip,
                                  const std::vector<double>& q)
{
  state.setJointGroupPositions(group, q);
  state.updateLinkTransforms();
  return state.getGlobalLinkTransform(kBase).inverse() * state.getGlobalLinkTransform(tip);
}

/// @brief Makes a case whose target poses are the tip poses of the target configurations.
/// @param model Robot model.
/// @param group Group name.
/// @param tip Tip link name.
/// @param targets Target configurations.
/// @param seeds Seed for each query.
/// @return The case.
Case makeCase(const moveit::core::RobotModelPtr& model, const std::string& group, const std::string& tip,
              std::vector<std::vector<double>> targets, std::vector<std::vector<double>> seeds)
{
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  Case result{ group, tip, std::move(targets), {}, std::move(seeds) };
  for (const std::vector<double>& q : result.targets)
  {
    result.poses.push_back(tf2::toMsg(getTipTransform(state, group, tip, q)));
  }
  return result;
}

/// @brief Runs searchPositionIK for every query of a case.
/// @param solver Solver, initialised for the group of the case.
/// @param model Robot model.
/// @param test_case Case to run.
/// @return The results.
Stats runCase(const kinematics::KinematicsBase& solver, const moveit::core::RobotModelPtr& model, const Case& test_case)
{
  const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(test_case.group);
  moveit::core::RobotState state(model);
  state.setToDefaultValues();

  Stats stats;
  std::vector<double> times_us;
  std::vector<double> seed_distances;
  int successes = 0;
  int same_branch = 0;
  for (std::size_t i = 0; i < test_case.poses.size(); ++i)
  {
    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    const auto start = std::chrono::steady_clock::now();
    const bool solved = solver.searchPositionIK(test_case.poses[i], test_case.seeds[i], kTimeout, solution, error_code);
    times_us.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count());
    if (!solved)
    {
      continue;
    }

    ++successes;
    seed_distances.push_back(jmg->distance(solution.data(), test_case.seeds[i].data()));
    if (!test_case.targets.empty())
    {
      const auto& joints = jmg->getActiveJointModels();
      bool is_same = true;
      for (std::size_t j = 0; j < joints.size(); ++j)
      {
        is_same = is_same && joints[j]->distance(&solution[j], &test_case.targets[i][j]) <= kSameBranchTolerance;
      }
      same_branch += is_same ? 1 : 0;
    }

    Eigen::Isometry3d target;
    tf2::fromMsg(test_case.poses[i], target);
    const Eigen::Isometry3d reached = getTipTransform(state, test_case.group, test_case.tip, solution);
    stats.max_position_error =
        std::max(stats.max_position_error, (reached.translation() - target.translation()).norm());
    stats.max_orientation_error = std::max(stats.max_orientation_error,
                                           Eigen::AngleAxisd(reached.linear().transpose() * target.linear()).angle());
  }

  const double count = static_cast<double>(test_case.poses.size());
  stats.success_percent = 100.0 * successes / count;
  stats.same_branch_percent = 100.0 * same_branch / count;
  stats.median_us = getPercentile(times_us, 0.5);
  stats.p95_us = getPercentile(times_us, 0.95);
  stats.median_seed_distance = getPercentile(seed_distances, 0.5);
  return stats;
}

/// @brief Gets the median number of solutions that getPositionIK with several solutions returns.
/// @param solver Solver, initialised for the group of the case.
/// @param test_case Case with reachable poses.
/// @return The median, or NaN if no query returned solutions.
double getMedianSolutionCount(const kinematics::KinematicsBase& solver, const Case& test_case)
{
  std::vector<double> counts;
  for (std::size_t i = 0; i < test_case.poses.size(); ++i)
  {
    std::vector<std::vector<double>> solutions;
    kinematics::KinematicsResult result;
    if (solver.getPositionIK({ test_case.poses[i] }, test_case.seeds[i], solutions, result,
                             kinematics::KinematicsQueryOptions()))
    {
      counts.push_back(static_cast<double>(solutions.size()));
    }
  }
  return getPercentile(counts, 0.5);
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const int num_poses = argc > 1 ? std::stoi(argv[1]) : 1000;

  const urdf::ModelInterfaceSharedPtr urdf = moveit::core::loadModelInterface("fanuc");
  const auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, kSrdf);
  const auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  const moveit::core::JointModelGroup& six = *model->getJointModelGroup("manipulator");
  const moveit::core::JointModelGroup& five = *model->getJointModelGroup("five_joints");

  std::mt19937 rng(42);
  const auto get_configurations = [&](const moveit::core::JointModelGroup& jmg) {
    std::vector<std::vector<double>> configurations;
    for (int i = 0; i < num_poses; ++i)
    {
      configurations.push_back(getRandomConfiguration(jmg, rng));
    }
    return configurations;
  };

  const Case reachable = makeCase(model, "manipulator", "tool0", get_configurations(six), get_configurations(six));

  std::vector<std::vector<double>> near_seeds = reachable.targets;
  std::uniform_real_distribution<double> noise(-kNearSeedNoise, kNearSeedNoise);
  for (std::vector<double>& seed : near_seeds)
  {
    for (double& value : seed)
    {
      value += noise(rng);
    }
    six.enforcePositionBounds(seed.data());
  }
  const Case near_seed = makeCase(model, "manipulator", "tool0", reachable.targets, near_seeds);

  std::vector<std::vector<double>> near_singular_targets = get_configurations(six);
  for (std::vector<double>& q : near_singular_targets)
  {
    q[kWristJointIndex] = std::uniform_real_distribution<double>(-kNearSingularRange, kNearSingularRange)(rng);
  }
  const Case near_singular = makeCase(model, "manipulator", "tool0", near_singular_targets, get_configurations(six));

  std::vector<std::vector<double>> singular_targets = get_configurations(six);
  for (std::vector<double>& q : singular_targets)
  {
    q[kWristJointIndex] = 0.0;
  }
  const Case singular = makeCase(model, "manipulator", "tool0", singular_targets, get_configurations(six));

  Case unreachable = reachable;
  unreachable.targets.clear();
  for (geometry_msgs::msg::Pose& pose : unreachable.poses)
  {
    pose.position.x += kUnreachableOffset;
  }

  const Case five_joints = makeCase(model, "five_joints", "link_5", get_configurations(five), get_configurations(five));

  // Cached KDL reads and writes its cache in this directory, so each run starts with an empty cache.
  char cache_template[] = "/tmp/benchmark_eaik_kinematics_XXXXXX";
  const std::string cache_path = mkdtemp(cache_template);
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({ { "robot_description_kinematics.manipulator.cached_ik_path", cache_path },
                                     { "robot_description_kinematics.five_joints.cached_ik_path", cache_path } });

  pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");
  std::vector<std::string> names;
  std::vector<std::string> skipped;
  std::vector<std::map<std::string, Stats>> results;
  std::vector<double> solution_counts;
  for (std::size_t p = 0; p < kPlugins.size(); ++p)
  {
    const std::string& plugin = kPlugins[p];
    if (!loader.isClassAvailable(plugin))
    {
      skipped.push_back(plugin + " (not installed)");
      continue;
    }

    // Each plugin gets its own node, because plugins declare parameters of the same name with different types.
    const auto node = std::make_shared<rclcpp::Node>("benchmark_" + std::to_string(p), node_options);
    std::map<std::string, Stats> stats;
    double solution_count = std::numeric_limits<double>::quiet_NaN();
    bool initialised = true;
    for (const std::string group : { "manipulator", "five_joints" })
    {
      const kinematics::KinematicsBasePtr solver = loader.createSharedInstance(plugin);
      const std::string tip = group == "manipulator" ? "tool0" : "link_5";
      if (!solver->initialize(node, *model, group, kBase, { tip }, 0.1))
      {
        initialised = false;
        break;
      }
      // MoveIt's plugin loader sets this from kinematics_solver_timeout. Some plugins use it in getPositionIK().
      solver->setDefaultTimeout(kTimeout);
      if (group == "five_joints")
      {
        stats["five_joints"] = runCase(*solver, model, five_joints);
        continue;
      }
      stats["reachable"] = runCase(*solver, model, reachable);
      stats["near_seed"] = runCase(*solver, model, near_seed);
      stats["near_singular"] = runCase(*solver, model, near_singular);
      stats["singular"] = runCase(*solver, model, singular);
      stats["unreachable"] = runCase(*solver, model, unreachable);
      solution_count = getMedianSolutionCount(*solver, reachable);
    }
    if (!initialised)
    {
      skipped.push_back(plugin + " (initialize() failed)");
      continue;
    }
    names.push_back(plugin);
    results.push_back(stats);
    solution_counts.push_back(solution_count);
  }
  std::filesystem::remove_all(cache_path);

  std::printf("Fanuc M-10iA, %d poses per case, timeout %.3f s, random seeds unless stated.\n", num_poses, kTimeout);
  for (const std::string& name : skipped)
  {
    std::printf("Skipped: %s\n", name.c_str());
  }

  std::printf("\n### Reachable poses\n\n");
  std::printf("| Plugin | Success | Median / p95 (us) | Max position error (m) | Max orientation error (rad) | "
              "Median seed distance (rad) | Solutions per pose |\n|---|---|---|---|---|---|---|\n");
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    const Stats& s = results[i].at("reachable");
    std::printf("| %s | %s %% | %s / %s | %s | %s | %s | %s |\n", names[i].c_str(),
                format("%.1f", s.success_percent).c_str(), format("%.1f", s.median_us).c_str(),
                format("%.1f", s.p95_us).c_str(), format("%.1e", s.max_position_error).c_str(),
                format("%.1e", s.max_orientation_error).c_str(), format("%.2f", s.median_seed_distance).c_str(),
                format("%.0f", solution_counts[i]).c_str());
  }

  std::printf("\n### Seed near the target configuration (+/-%.2f rad per joint)\n\n", kNearSeedNoise);
  std::printf("| Plugin | Success | Same branch as target | Median / p95 (us) |\n|---|---|---|---|\n");
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    const Stats& s = results[i].at("near_seed");
    std::printf("| %s | %s %% | %s %% | %s / %s |\n", names[i].c_str(), format("%.1f", s.success_percent).c_str(),
                format("%.1f", s.same_branch_percent).c_str(), format("%.1f", s.median_us).c_str(),
                format("%.1f", s.p95_us).c_str());
  }

  std::printf("\n### Wrist singularity (joint_5 within +/-%.0e rad, and exactly 0)\n\n", kNearSingularRange);
  std::printf("| Plugin | Near: success | Near: median / p95 (us) | Exact: success | Exact: median / p95 (us) | "
              "Max position error (m) |\n|---|---|---|---|---|---|\n");
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    const Stats& n = results[i].at("near_singular");
    const Stats& e = results[i].at("singular");
    std::printf("| %s | %s %% | %s / %s | %s %% | %s / %s | %s |\n", names[i].c_str(),
                format("%.1f", n.success_percent).c_str(), format("%.1f", n.median_us).c_str(),
                format("%.1f", n.p95_us).c_str(), format("%.1f", e.success_percent).c_str(),
                format("%.1f", e.median_us).c_str(), format("%.1f", e.p95_us).c_str(),
                format("%.1e", std::max(n.max_position_error, e.max_position_error)).c_str());
  }

  std::printf("\n### Unreachable poses (%.0f m away)\n\n", kUnreachableOffset);
  std::printf("| Plugin | Wrongly solved | Median / p95 (us) |\n|---|---|---|\n");
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    const Stats& s = results[i].at("unreachable");
    std::printf("| %s | %s %% | %s / %s |\n", names[i].c_str(), format("%.1f", s.success_percent).c_str(),
                format("%.1f", s.median_us).c_str(), format("%.1f", s.p95_us).c_str());
  }

  std::printf("\n### 5-joint group (base_link to link_5), reachable poses\n\n");
  std::printf("| Plugin | Success | Median / p95 (us) | Max position error (m) | Max orientation error (rad) |\n"
              "|---|---|---|---|---|\n");
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    const Stats& s = results[i].at("five_joints");
    std::printf("| %s | %s %% | %s / %s | %s | %s |\n", names[i].c_str(), format("%.1f", s.success_percent).c_str(),
                format("%.1f", s.median_us).c_str(), format("%.1f", s.p95_us).c_str(),
                format("%.1e", s.max_position_error).c_str(), format("%.1e", s.max_orientation_error).c_str());
  }

  rclcpp::shutdown();
  // MoveIt's logger keeps a static node. Its destructor at exit calls rmw_zenoh after zenoh has shut down,
  // which aborts the process. Skip the static destructors.
  std::fflush(stdout);
  std::quick_exit(0);
}
