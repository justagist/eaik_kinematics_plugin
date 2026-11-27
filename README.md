# eaik_kinematics_plugin

A MoveIt 2 kinematics plugin that solves inverse kinematics (IK) in closed form with
[EAIK](https://github.com/OstermD/EAIK). EAIK finds all IK solutions of an arm with up to 6
revolute joints in microseconds, without a seed and without iterations.

The repository has two packages:

| Package | Contents |
|---|---|
| `eaik_vendor` | Builds and installs the EAIK C++ library at a pinned commit. If CMake finds an installed EAIK package of version 1.2.2 or newer, it builds nothing. |
| `eaik_kinematics_plugin` | The MoveIt plugin `eaik_kinematics_plugin/EAIKKinematicsPlugin`, its tests and a benchmark. |

One branch supports ROS 2 Humble, Jazzy, Lyrical and Rolling.

## Supported groups

The plugin accepts a group if all of these conditions are true:

- The group is a chain of 1 to 6 revolute joints. Continuous joints are permitted. Prismatic, mimic
  and multi-DOF joints are not.
- The group has one tip frame.
- Only fixed joints are between the base frame and the first joint, and between the last joint and
  the tip frame.
- EAIK knows a closed-form decomposition for the arm, for example a spherical wrist, or three
  parallel or intersecting axes. The
  [EAIK README](https://github.com/OstermD/EAIK) lists the supported kinematic families.
- EAIK solves the group. EAIK accepts some groups that it cannot solve, so `initialize()` does a
  check: it samples 100 configurations, and EAIK must get each one back from its pose.

If one condition is false, `initialize()` fails and logs the reason. Use a different solver for
that group, for example KDL.

The check rejects these groups:

- An arm whose URDF has rounded values. EAIK finds parallel and intersecting axes with tolerances
  near 1e-12. An angle of `1.5708` in place of pi/2 tilts an axis by 3.7e-6 rad, and EAIK then
  selects a kinematic family that does not solve the arm. Give exact values in the URDF, for
  example `${pi/2}` in xacro.
- A group of 1 or 2 joints whose tip frame is on the axis of the last joint. EAIK solves these
  groups from the tip position only.
- Some groups of 4 joints, for example the first 4 joints of a UR-type arm.

A group of 4 or more joints with all axes parallel is redundant, and is also rejected.

A group with fewer than 6 joints cannot reach all positions and orientations. The plugin gives a
solution only if the arm can reach the pose exactly (within the tolerances). Thus most pose goals
for such a group fail. EAIK has no position-only IK in its public interface, so the plugin cannot
relax the orientation like the `position_only_ik` parameter of KDL.

## Usage

Set the solver for the group in `kinematics.yaml`:

```yaml
manipulator:
  kinematics_solver: eaik_kinematics_plugin/EAIKKinematicsPlugin
  kinematics_solver_timeout: 0.005
```

### Parameters

The parameters are in `robot_description_kinematics.<group>`.

| Parameter | Default | Description |
|---|---|---|
| `position_tolerance` | `1e-5` | Maximum position error in metres of a solution, checked with forward kinematics. |
| `orientation_tolerance` | `1e-5` | Maximum orientation error in radians of a solution, checked with forward kinematics. |

## Behaviour

- `getPositionIK()` with several solutions returns all EAIK solutions that reach the pose within
  the tolerances and obey the joint limits, sorted by distance to the seed. If a joint can turn more
  than 2 pi, each equal value modulo 2 pi within its limits is a separate solution.
- `searchPositionIK()` solves once, then gives the solutions to the solution callback in order of
  distance to the seed. It returns the first solution that the callback accepts. If no solution is
  accepted, the error code is `NO_IK_SOLUTION`. The plugin does not use the timeout, because it does
  not iterate.
- EAIK also returns least-squares solutions for poses that are out of reach. The plugin keeps only
  the solutions whose forward kinematics are within the tolerances.
- At a singular configuration, the solutions are a continuous set and EAIK returns one of them.
  For example, with joint 5 of a spherical-wrist arm at 0, only the sum of joints 4 and 6 is
  fixed, and EAIK sets joint 4 to 0. The solution can then be far from the seed in these two
  joints, and a search with consistency limits can fail. A Cartesian path that stays at such a
  configuration can thus stop or jump.
- `getPositionFK()` uses the EAIK model, and supports the tip frame only. This model is the one that
  the IK solves, so FK and IK agree.
- `return_approximate_solution` is not used.
- The const methods are safe for concurrent calls: they only read the data that `initialize()`
  writes, and EAIK keeps no mutable state.

## Building

Clone the repository into the `src` directory of a colcon workspace, then:

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to eaik_kinematics_plugin
```

### About eaik_vendor

EAIK is not in rosdep, so `eaik_vendor` builds it with `ament_cmake_vendor_package`:

- It fetches EAIK from the `packaged-main` branch of the fork
  [justagist/EAIK](https://github.com/justagist/EAIK) (commit `d3887e1`) at build time, so the
  build needs network access. The fork is EAIK v1.2.2 with three fixes:
  - `CPP/src` installs a library with the CMake target `EAIK::EAIK`, and uses the system Eigen.
  - Joint 6 is correct at the wrist singularity of spherical-wrist arms (axes 4 and 6 aligned).
    Without this fix, the plugin finds no solution for about 4 % of the poses at the singularity
    of the Fanuc arm.
  - Parallel axes that point in opposite directions are solved. Without this fix, EAIK solves no
    pose of an arm such as the AUBO iS10, whose axes 2, 3 and 4 are -y, +y and -y.
- The fetch also clones the bundled Eigen submodule (about 290 MB), because `ament_vendor` always
  clones submodules. The build does not use it.
- If CMake finds EAIK 1.2.2 or newer (`find_package(EAIK)`), `eaik_vendor` builds nothing and the
  plugin uses that installation. A pip install of EAIK is not found, because it has no CMake
  package. An installation that does not come from the fork may not have the two solver fixes.
  Set `FORCE_BUILD_VENDOR_PKG=ON` to build EAIK anyway.

## Tests

```bash
colcon build --packages-up-to eaik_kinematics_plugin
colcon test --packages-select eaik_kinematics_plugin
colcon test-result --verbose
```

The tests use the Fanuc and Panda arms from `moveit_resources`. They check IK against MoveIt's
forward kinematics for random configurations, configurations at the wrist singularity, the
solution order, the 2 pi variants, the solution callback, consistency limits, unreachable poses and a
5-joint group. They also check that `initialize()` rejects a 7-joint group, several tip frames, a
reversed chain without a decomposition, an arm with axes that are not exactly parallel, a single
joint with the tip on its axis, and a 4-joint chain with parallel axes.

## Benchmark

`benchmark_eaik_kinematics` compares this plugin with other kinematics plugins on the Fanuc arm from
`moveit_resources`:

| Plugin | Package |
|---|---|
| `eaik_kinematics_plugin/EAIKKinematicsPlugin` | `eaik_kinematics_plugin` |
| `kdl_kinematics_plugin/KDLKinematicsPlugin` | `moveit_kinematics` |
| `cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin` | `moveit_kinematics` |
| `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` | `trac_ik_kinematics_plugin` |
| `pick_ik/PickIkPlugin` | `pick_ik` |

The benchmark skips a plugin that is not installed. `rosdep` does not install TRAC-IK or pick_ik for
this package. To include them, install them, for example
`apt install ros-$ROS_DISTRO-trac-ik-kinematics-plugin ros-$ROS_DISTRO-pick-ik`.

Each case has 1000 queries to `searchPositionIK()` with a timeout of 15 ms, the
`kinematics_solver_timeout` of the Fanuc MoveIt config. The target poses are the forward kinematics
of random configurations within the joint limits. The seeds are random configurations, except in the
case with seeds near the target. MoveIt forward kinematics measures the errors of all plugins. All
plugins use their default parameters. Cached KDL starts with an empty cache.

The benchmark is built with the tests (`BUILD_TESTING=ON`, the default of colcon):

```bash
ros2 run eaik_kinematics_plugin benchmark_eaik_kinematics 1000
```

Results on an Intel Core i7-14650HX, ROS 2 Jazzy, Release build:

#### Reachable poses

| Plugin | Success | Median / p95 (us) | Max position error (m) | Max orientation error (rad) | Median seed distance (rad) | Solutions per pose |
|---|---|---|---|---|---|---|
| eaik_kinematics_plugin/EAIKKinematicsPlugin | 100.0 % | 14.8 / 18.5 | 4.9e-15 | 4.9e-14 | 7.62 | 16 |
| kdl_kinematics_plugin/KDLKinematicsPlugin | 93.1 % | 2024.9 / 15209.3 | 1.0e-05 | 9.6e-06 | 13.63 | 1 |
| cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin | 99.4 % | 313.5 / 20306.5 | 9.7e-06 | 1.0e-05 | 13.60 | 1 |
| trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin | 99.9 % | 15151.8 / 15237.8 | 1.5e-05 | 1.3e-05 | 7.71 | 1 |
| pick_ik/PickIkPlugin | 74.5 % | 8509.4 / 16528.2 | 9.9e-04 | 1.0e-03 | 12.97 | - |

#### Seed near the target configuration (+/-0.05 rad per joint)

| Plugin | Success | Same branch as target | Median / p95 (us) |
|---|---|---|---|
| eaik_kinematics_plugin/EAIKKinematicsPlugin | 100.0 % | 100.0 % | 14.9 / 18.3 |
| kdl_kinematics_plugin/KDLKinematicsPlugin | 99.4 % | 98.7 % | 43.3 / 71.2 |
| cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin | 99.8 % | 7.4 % | 318.5 / 14912.9 |
| trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin | 100.0 % | 100.0 % | 15163.1 / 15234.4 |
| pick_ik/PickIkPlugin | 100.0 % | 96.7 % | 1885.8 / 5469.4 |

#### Wrist singularity (joint_5 within +/-1e-03 rad, and exactly 0)

| Plugin | Near: success | Near: median / p95 (us) | Exact: success | Exact: median / p95 (us) | Max position error (m) |
|---|---|---|---|---|---|
| eaik_kinematics_plugin/EAIKKinematicsPlugin | 100.0 % | 15.5 / 18.4 | 100.0 % | 13.9 / 17.0 | 2.6e-09 |
| kdl_kinematics_plugin/KDLKinematicsPlugin | 75.3 % | 5421.8 / 19199.8 | 96.6 % | 1099.8 / 11438.3 | 1.0e-05 |
| cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin | 96.8 % | 757.8 / 73792.9 | 100.0 % | 143.5 / 7547.5 | 1.0e-05 |
| trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin | 99.9 % | 15160.2 / 15237.8 | 100.0 % | 15150.6 / 15225.1 | 1.6e-05 |
| pick_ik/PickIkPlugin | 88.1 % | 5432.6 / 16174.3 | 88.9 % | 5418.9 / 16005.7 | 1.0e-03 |

#### Unreachable poses (10 m away)

| Plugin | Wrongly solved | Median / p95 (us) |
|---|---|---|
| eaik_kinematics_plugin/EAIKKinematicsPlugin | 0.0 % | 4.9 / 5.1 |
| kdl_kinematics_plugin/KDLKinematicsPlugin | 0.0 % | 15125.0 / 15541.8 |
| cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin | 0.0 % | 92253.4 / 96135.7 |
| trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin | 0.0 % | 15156.0 / 15227.0 |
| pick_ik/PickIkPlugin | 0.0 % | 15993.7 / 16792.0 |

#### 5-joint group (base_link to link_5), reachable poses

| Plugin | Success | Median / p95 (us) | Max position error (m) | Max orientation error (rad) |
|---|---|---|---|---|
| eaik_kinematics_plugin/EAIKKinematicsPlugin | 100.0 % | 23.0 / 24.0 | 2.2e-14 | 1.1e-13 |
| kdl_kinematics_plugin/KDLKinematicsPlugin | 64.1 % | 7664.0 / 18399.7 | 9.7e-06 | 9.8e-06 |
| cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin | 96.9 % | 6116.5 / 71325.5 | 9.9e-06 | 9.9e-06 |
| trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin | 100.0 % | 15158.9 / 15234.7 | 1.5e-05 | 1.5e-05 |
| pick_ik/PickIkPlugin | 76.6 % | 8704.7 / 16279.1 | 1.0e-03 | 9.9e-04 |

Notes on the results:

- KDL, Cached KDL and TRAC-IK give one solution per pose, because they use the default
  multi-solution `getPositionIK()` of `KinematicsBase`. pick_ik returned no solution from the
  multi-solution `getPositionIK()` for any pose. The cause is not known.
- TRAC-IK uses its default solve type `Distance`. It searches for the full timeout for the solution
  nearest to the seed, so its time is always near 15 ms. The solve type `Speed` returns the first
  solution.
- pick_ik uses its default thresholds of 1e-3 m and 1e-3 rad, which explains its larger errors.
- Cached KDL returns a solution from its cache, which is often on a different branch from the seed. It
  can also take longer than the timeout. On the 5-joint group, Cached KDL logged the error "Seed
  state must have size 5 instead of size 6" 36 times.
- On the 5-joint group, the targets are full poses. A MoveIt config usually sets `position_only_ik`
  for KDL on such a group, so the KDL result there is not typical for a configured arm.

## Licence

This repository is licensed under the BSD 3-Clause License, the same licence as EAIK and
MoveIt. See [LICENSE](LICENSE).

`eaik_vendor` downloads and builds third-party code with these licences:

| Project | Licence |
|---|---|
| [EAIK](https://github.com/OstermD/EAIK) | BSD 3-Clause, Copyright (c) 2025 Daniel Ostermeier |
| [ik-geo](https://github.com/OstermD/ik-geo) (EAIK submodule, fork of rpiRobotics/ik-geo) | BSD 3-Clause, Copyright (c) 2022 rpiRobotics |
