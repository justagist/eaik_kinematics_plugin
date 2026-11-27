// Copyright 2026 Saif Sidhik
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
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

#include <srdfdom/model.h>

namespace
{
constexpr char kPluginName[] = "eaik_kinematics_plugin/EAIKKinematicsPlugin";
constexpr double kPoseTolerance = 1e-5;
constexpr double kJointTolerance = 1e-6;
constexpr int kNumSamples = 1000;
// A 5-joint group of the Fanuc arm: the 6-joint arm without its last joint.
constexpr char kFiveJointSrdf[] = R"(<?xml version="1.0"?>
<robot name="fanuc">
  <group name="five_joints">
    <chain base_link="base_link" tip_link="link_5"/>
  </group>
</robot>)";

using moveit_msgs::msg::MoveItErrorCodes;

/// @brief Creates a solver from the plugin and initialises it.
/// @param loader Loader that must outlive the solver.
/// @param robot_model Robot model to initialise with.
/// @param group Group name.
/// @param base Base frame.
/// @param tips Tip frames.
/// @return The solver and the result of initialize().
std::pair<kinematics::KinematicsBasePtr, bool> createSolver(pluginlib::ClassLoader<kinematics::KinematicsBase>& loader,
                                                            const moveit::core::RobotModel& robot_model,
                                                            const std::string& group, const std::string& base,
                                                            const std::vector<std::string>& tips)
{
  const auto node = std::make_shared<rclcpp::Node>("eaik_kinematics_plugin_test");
  kinematics::KinematicsBasePtr solver = loader.createSharedInstance(kPluginName);
  const bool initialised = solver->initialize(node, robot_model, group, base, tips, 0.1);
  return { solver, initialised };
}

class EAIKFanucTest : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("fanuc");
    initialiseSolver("manipulator", "tool0");
  }

  /// @brief Creates and initialises the solver for a group of robot_model_ with base frame base_link.
  void initialiseSolver(const std::string& group, const std::string& tip)
  {
    jmg_ = robot_model_->getJointModelGroup(group);
    tip_ = tip;
    bool initialised = false;
    std::tie(solver_, initialised) = createSolver(loader_, *robot_model_, group, "base_link", { tip });
    ASSERT_TRUE(initialised);
  }

  /// @brief Draws joint values uniformly within the joint limits.
  std::vector<double> getRandomConfiguration()
  {
    std::vector<double> q;
    for (const moveit::core::JointModel* joint : jmg_->getActiveJointModels())
    {
      const moveit::core::VariableBounds& bounds = joint->getVariableBounds().front();
      q.push_back(std::uniform_real_distribution<double>(bounds.min_position_, bounds.max_position_)(rng_));
    }
    return q;
  }

  /// @brief Computes the tip pose in the base frame with MoveIt, independently of EAIK.
  Eigen::Isometry3d getTipTransform(const std::vector<double>& q) const
  {
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    state.setJointGroupPositions(jmg_, q);
    state.updateLinkTransforms();
    return state.getGlobalLinkTransform("base_link").inverse() * state.getGlobalLinkTransform(tip_);
  }

  /// @brief Checks that the solution reaches the target and obeys the joint limits.
  void expectReaches(const std::vector<double>& solution, const Eigen::Isometry3d& target) const
  {
    ASSERT_EQ(solution.size(), jmg_->getActiveJointModels().size());
    EXPECT_TRUE(jmg_->satisfiesPositionBounds(solution.data()));
    const Eigen::Isometry3d reached = getTipTransform(solution);
    EXPECT_LT((reached.translation() - target.translation()).norm(), kPoseTolerance);
    EXPECT_LT(Eigen::AngleAxisd(reached.linear().transpose() * target.linear()).angle(), kPoseTolerance);
  }

  /// @brief Checks whether two configurations are equal joint by joint.
  static bool isSameConfiguration(const std::vector<double>& a, const std::vector<double>& b)
  {
    for (std::size_t i = 0; i < a.size(); ++i)
    {
      if (std::abs(a[i] - b[i]) > kJointTolerance)
      {
        return false;
      }
    }
    return true;
  }

  // A configuration away from singularities, used where solution branches must stay far apart.
  const std::vector<double> regular_q_{ 0.3, 0.2, -0.4, 0.5, 0.8, 1.0 };

  pluginlib::ClassLoader<kinematics::KinematicsBase> loader_{ "moveit_core", "kinematics::KinematicsBase" };
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup* jmg_ = nullptr;
  std::string tip_;
  kinematics::KinematicsBasePtr solver_;
  std::mt19937 rng_{ 42 };
};

TEST_F(EAIKFanucTest, SolvesRandomReachablePosesFromRandomSeeds)
{
  for (int i = 0; i < kNumSamples; ++i)
  {
    const Eigen::Isometry3d target = getTipTransform(getRandomConfiguration());
    std::vector<double> solution;
    MoveItErrorCodes error_code;
    ASSERT_TRUE(solver_->getPositionIK(tf2::toMsg(target), getRandomConfiguration(), solution, error_code));
    EXPECT_EQ(error_code.val, MoveItErrorCodes::SUCCESS);
    expectReaches(solution, target);
  }
}

TEST_F(EAIKFanucTest, AllSolutionsContainTheGeneratingConfiguration)
{
  for (int i = 0; i < kNumSamples; ++i)
  {
    const std::vector<double> q = getRandomConfiguration();
    const Eigen::Isometry3d target = getTipTransform(q);
    std::vector<std::vector<double>> solutions;
    kinematics::KinematicsResult result;
    ASSERT_TRUE(
        solver_->getPositionIK({ tf2::toMsg(target) }, q, solutions, result, kinematics::KinematicsQueryOptions()));
    EXPECT_EQ(result.kinematic_error, kinematics::KinematicErrors::OK);
    for (const std::vector<double>& solution : solutions)
    {
      expectReaches(solution, target);
    }
    EXPECT_TRUE(isSameConfiguration(solutions.front(), q)) << "the solution nearest to the seed comes first";
  }
}

TEST_F(EAIKFanucTest, SolvesPosesAtTheWristSingularity)
{
  // With joint_5 at 0, the axes of joint_4 and joint_6 are aligned, so only their sum is fixed.
  for (int i = 0; i < kNumSamples; ++i)
  {
    std::vector<double> q = getRandomConfiguration();
    q[4] = 0.0;
    const Eigen::Isometry3d target = getTipTransform(q);
    std::vector<double> solution;
    MoveItErrorCodes error_code;
    ASSERT_TRUE(solver_->getPositionIK(tf2::toMsg(target), getRandomConfiguration(), solution, error_code));
    expectReaches(solution, target);
  }
}

TEST_F(EAIKFanucTest, SearchReturnsTheSolutionNearestToTheSeed)
{
  for (int i = 0; i < kNumSamples; ++i)
  {
    const std::vector<double> q = getRandomConfiguration();
    std::vector<double> solution;
    MoveItErrorCodes error_code;
    ASSERT_TRUE(solver_->searchPositionIK(tf2::toMsg(getTipTransform(q)), q, 0.1, solution, error_code));
    EXPECT_TRUE(isSameConfiguration(solution, q));
  }
}

TEST_F(EAIKFanucTest, ReturnsEveryTwoPiVariantWithinTheJointLimits)
{
  // joint_6 has limits of +/-6.28, so 1.0 - 2 pi is also within them, but 1.0 + 2 pi is not.
  std::vector<double> wrapped = regular_q_;
  wrapped[5] -= 2.0 * M_PI;

  std::vector<std::vector<double>> solutions;
  kinematics::KinematicsResult result;
  ASSERT_TRUE(solver_->getPositionIK({ tf2::toMsg(getTipTransform(regular_q_)) }, regular_q_, solutions, result,
                                     kinematics::KinematicsQueryOptions()));
  const auto contains = [&](const std::vector<double>& q) {
    return std::any_of(solutions.begin(), solutions.end(),
                       [&](const std::vector<double>& solution) { return isSameConfiguration(solution, q); });
  };
  EXPECT_TRUE(contains(regular_q_));
  EXPECT_TRUE(contains(wrapped));
  for (const std::vector<double>& solution : solutions)
  {
    EXPECT_TRUE(jmg_->satisfiesPositionBounds(solution.data()));
  }
}

TEST_F(EAIKFanucTest, CallbackSelectsTheFirstAcceptedSolution)
{
  const geometry_msgs::msg::Pose pose = tf2::toMsg(getTipTransform(regular_q_));
  std::vector<std::vector<double>> solutions;
  kinematics::KinematicsResult result;
  ASSERT_TRUE(solver_->getPositionIK({ pose }, regular_q_, solutions, result, kinematics::KinematicsQueryOptions()));
  ASSERT_GT(solutions.size(), 1u);

  std::size_t calls = 0;
  const auto accept_last = [&](const geometry_msgs::msg::Pose&, const std::vector<double>& candidate,
                               MoveItErrorCodes& error_code) {
    ++calls;
    error_code.val =
        isSameConfiguration(candidate, solutions.back()) ? MoveItErrorCodes::SUCCESS : MoveItErrorCodes::NO_IK_SOLUTION;
  };
  std::vector<double> solution;
  MoveItErrorCodes error_code;
  ASSERT_TRUE(solver_->searchPositionIK(pose, regular_q_, 0.1, solution, accept_last, error_code));
  EXPECT_EQ(error_code.val, MoveItErrorCodes::SUCCESS);
  EXPECT_TRUE(isSameConfiguration(solution, solutions.back()));
  EXPECT_EQ(calls, solutions.size());
}

TEST_F(EAIKFanucTest, FailsWhenTheCallbackRejectsEverySolution)
{
  const auto reject = [](const geometry_msgs::msg::Pose&, const std::vector<double>&, MoveItErrorCodes& error_code) {
    error_code.val = MoveItErrorCodes::GOAL_IN_COLLISION;
  };
  std::vector<double> solution;
  MoveItErrorCodes error_code;
  EXPECT_FALSE(solver_->searchPositionIK(tf2::toMsg(getTipTransform(regular_q_)), regular_q_, 0.1, solution, reject,
                                         error_code));
  EXPECT_EQ(error_code.val, MoveItErrorCodes::NO_IK_SOLUTION);
}

TEST_F(EAIKFanucTest, ObeysConsistencyLimits)
{
  const geometry_msgs::msg::Pose pose = tf2::toMsg(getTipTransform(regular_q_));
  const std::vector<double> limits(6, 0.1);

  std::vector<double> near_seed = regular_q_;
  for (double& value : near_seed)
  {
    value += 0.05;
  }
  std::vector<double> solution;
  MoveItErrorCodes error_code;
  ASSERT_TRUE(solver_->searchPositionIK(pose, near_seed, 0.1, limits, solution, error_code));
  EXPECT_TRUE(isSameConfiguration(solution, regular_q_));

  std::vector<double> far_seed = regular_q_;
  far_seed[0] += 0.5;
  EXPECT_FALSE(solver_->searchPositionIK(pose, far_seed, 0.1, limits, solution, error_code));
  EXPECT_EQ(error_code.val, MoveItErrorCodes::NO_IK_SOLUTION);
}

TEST_F(EAIKFanucTest, FailsOnUnreachablePoseWithoutWaitingForTheTimeout)
{
  Eigen::Isometry3d target = getTipTransform(regular_q_);
  target.translation().x() += 10.0;
  const geometry_msgs::msg::Pose pose = tf2::toMsg(target);

  std::vector<double> solution;
  MoveItErrorCodes error_code;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(solver_->searchPositionIK(pose, regular_q_, 1.0, solution, error_code));
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(100));
  EXPECT_EQ(error_code.val, MoveItErrorCodes::NO_IK_SOLUTION);

  std::vector<std::vector<double>> solutions;
  kinematics::KinematicsResult result;
  EXPECT_FALSE(solver_->getPositionIK({ pose }, regular_q_, solutions, result, kinematics::KinematicsQueryOptions()));
  EXPECT_EQ(result.kinematic_error, kinematics::KinematicErrors::NO_SOLUTION);
  EXPECT_TRUE(solutions.empty());
}

TEST_F(EAIKFanucTest, ForwardKinematicsMatchesRobotState)
{
  for (int i = 0; i < kNumSamples; ++i)
  {
    const std::vector<double> q = getRandomConfiguration();
    std::vector<geometry_msgs::msg::Pose> poses;
    ASSERT_TRUE(solver_->getPositionFK({ "tool0" }, q, poses));
    Eigen::Isometry3d reached;
    tf2::fromMsg(poses.front(), reached);
    const Eigen::Isometry3d expected = getTipTransform(q);
    EXPECT_LT((reached.translation() - expected.translation()).norm(), kPoseTolerance);
    EXPECT_LT(Eigen::AngleAxisd(reached.linear().transpose() * expected.linear()).angle(), kPoseTolerance);
  }
}

class EAIKFanucFiveJointTest : public EAIKFanucTest
{
protected:
  void SetUp() override
  {
    const urdf::ModelInterfaceSharedPtr urdf = moveit::core::loadModelInterface("fanuc");
    const auto srdf = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*urdf, kFiveJointSrdf));
    robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    initialiseSolver("five_joints", "link_5");
  }
};

TEST_F(EAIKFanucFiveJointTest, SolvesExactlyReachablePoses)
{
  for (int i = 0; i < kNumSamples; ++i)
  {
    const Eigen::Isometry3d target = getTipTransform(getRandomConfiguration());
    std::vector<double> solution;
    MoveItErrorCodes error_code;
    ASSERT_TRUE(solver_->getPositionIK(tf2::toMsg(target), getRandomConfiguration(), solution, error_code));
    expectReaches(solution, target);
  }
}

TEST_F(EAIKFanucFiveJointTest, FailsWhenOnlyTheOrientationIsOutOfReach)
{
  const std::vector<double> q{ 0.3, 0.2, -0.4, 0.5, 0.8 };
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(jmg_, q);
  state.updateLinkTransforms();
  const Eigen::Isometry3d base_inverse = state.getGlobalLinkTransform("base_link").inverse();
  const auto get_axis = [&](const std::string& joint_name) {
    const auto* joint = static_cast<const moveit::core::RevoluteJointModel*>(robot_model_->getJointModel(joint_name));
    return Eigen::Vector3d((base_inverse * state.getGlobalLinkTransform(joint->getChildLinkModel())).linear() *
                           joint->getAxis());
  };

  // link_5 is at the intersection of the axes of joints 4 and 5. With link_5 at a fixed position, these joints
  // can only turn it about their own axes, so a turn about the normal of both axes is out of reach.
  Eigen::Isometry3d target = getTipTransform(q);
  const Eigen::Vector3d normal = get_axis("joint_4").cross(get_axis("joint_5")).normalized();
  target.linear() = Eigen::AngleAxisd(0.1, normal) * target.linear();

  std::vector<double> solution;
  MoveItErrorCodes error_code;
  EXPECT_FALSE(solver_->searchPositionIK(tf2::toMsg(target), q, 0.1, solution, error_code));
  EXPECT_EQ(error_code.val, MoveItErrorCodes::NO_IK_SOLUTION);
}

TEST(EAIKInitialiseTest, RejectsSevenJointGroup)
{
  pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");
  const moveit::core::RobotModelPtr panda = moveit::core::loadTestingRobotModel("panda");
  EXPECT_FALSE(createSolver(loader, *panda, "panda_arm", "panda_link0", { "panda_link8" }).second);
}

TEST(EAIKInitialiseTest, RejectsMultipleTipFrames)
{
  pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");
  const moveit::core::RobotModelPtr fanuc = moveit::core::loadTestingRobotModel("fanuc");
  EXPECT_FALSE(createSolver(loader, *fanuc, "manipulator", "base_link", { "tool0", "link_5" }).second);
}
}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  // MoveIt's logger keeps a static node. Its destructor at exit calls rmw_zenoh after zenoh has shut down,
  // which aborts the process. Skip the static destructors.
  std::fflush(stdout);
  std::quick_exit(result);
}
