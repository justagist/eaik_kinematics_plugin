# eaik_kinematics_plugin

A MoveIt 2 kinematics plugin that solves inverse kinematics (IK) in closed form with
[EAIK](https://github.com/OstermD/EAIK). EAIK finds all IK solutions of an arm with up to 6
revolute joints in microseconds, without a seed and without iterations.

The repository has two packages:

| Package | Contents |
|---|---|
| `eaik_vendor` | Builds and installs the EAIK C++ library at a pinned commit. If EAIK is already installed, it builds nothing. |
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

If one condition is false, `initialize()` fails and logs the reason. Use a different solver for
that group, for example KDL.

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
- `getPositionFK()` uses the EAIK model, and supports the tip frame only. This model is the one that
  the IK solves, so FK and IK agree. It also does not allocate memory, and is safe for concurrent
  calls.
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

- It fetches EAIK at commit `3d973f6` (v1.2.2) at build time, so the build needs network access.
- The EAIK CMake files do not install a library. The patch in `eaik_vendor/patches` makes `CPP/src`
  an installable shared library with the CMake target `EAIK::EAIK`, and uses the system Eigen in
  place of the bundled Eigen. This change will be proposed upstream.
- The fetch also clones the bundled Eigen submodule (about 290 MB), because `ament_vendor` always
  clones submodules. The build does not use it.
- If CMake finds EAIK 1.2.2 or newer (`find_package(EAIK)`), `eaik_vendor` builds nothing and the
  plugin uses that installation. Set `FORCE_BUILD_VENDOR_PKG=ON` to build EAIK anyway.

## Tests

```bash
colcon build --packages-up-to eaik_kinematics_plugin
colcon test --packages-select eaik_kinematics_plugin
colcon test-result --verbose
```

The tests use the Fanuc and Panda arms from `moveit_resources`. They check IK against MoveIt's
forward kinematics for random configurations, the solution order, the 2 pi variants, the solution
callback, consistency limits, unreachable poses, a 5-joint group, and that a 7-joint group and
several tip frames are rejected.

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

Each case has 1000 queries to `searchPositionIK()` with a timeout of 5 ms, the
`kinematics_solver_timeout` of the Fanuc MoveIt config. The target poses are the forward kinematics
of random configurations within the joint limits. The seeds are random configurations, except in the
case with seeds near the target. MoveIt forward kinematics measures the errors of all plugins. All
plugins use their default parameters. Cached KDL starts with an empty cache.

```bash
ros2 run eaik_kinematics_plugin benchmark_eaik_kinematics 1000
```

Results on an Intel Core i7-14650HX, ROS 2 Jazzy, Release build:

#### Reachable poses

| Plugin | Success | Median / p95 (us) | Max position error (m) | Max orientation error (rad) | Median seed distance (rad) | Solutions per pose |
|---|---|---|---|---|---|---|
| eaik_kinematics_plugin/EAIKKinematicsPlugin | 100.0 % | 14.4 / 17.7 | 4.9e-15 | 7.7e-14 | 7.62 | 16 |
| kdl_kinematics_plugin/KDLKinematicsPlugin | 70.3 % | 1817.0 / 7743.3 | 9.4e-06 | 9.9e-06 | 13.10 | 1 |
| cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin | 98.6 % | 405.1 / 20811.5 | 9.8e-06 | 1.0e-05 | 14.30 | 1 |
| trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin | 99.8 % | 5144.0 / 5253.7 | 1.4e-05 | 1.4e-05 | 7.72 | 1 |
| pick_ik/PickIkPlugin | 42.1 % | 5260.1 / 6631.1 | 9.9e-04 | 9.9e-04 | 12.42 | - |

#### Seed near the target configuration (+/-0.05 rad per joint)

| Plugin | Success | Same branch as target | Median / p95 (us) |
|---|---|---|---|
| eaik_kinematics_plugin/EAIKKinematicsPlugin | 100.0 % | 100.0 % | 14.6 / 18.4 |
| kdl_kinematics_plugin/KDLKinematicsPlugin | 98.3 % | 98.3 % | 43.0 / 59.8 |
| cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin | 99.7 % | 8.1 % | 26.3 / 117.9 |
| trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin | 100.0 % | 100.0 % | 5133.6 / 5243.2 |
| pick_ik/PickIkPlugin | 97.1 % | 92.2 % | 1844.7 / 5307.6 |

#### Wrist singularity (joint_5 within +/-1e-03 rad, and exactly 0)

| Plugin | Near: success | Near: median / p95 (us) | Exact: success | Exact: median / p95 (us) | Max position error (m) |
|---|---|---|---|---|---|
| eaik_kinematics_plugin/EAIKKinematicsPlugin | 100.0 % | 15.2 / 17.9 | 96.0 % | 12.9 / 15.4 | 6.5e-12 |
| kdl_kinematics_plugin/KDLKinematicsPlugin | 51.5 % | 4175.6 / 9514.9 | 81.1 % | 1072.7 / 6722.5 | 9.9e-06 |
| cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin | 90.3 % | 640.2 / 50068.1 | 99.5 % | 132.4 / 12033.0 | 1.0e-05 |
| trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin | 99.6 % | 5126.1 / 5236.6 | 100.0 % | 5141.7 / 5241.6 | 1.5e-05 |
| pick_ik/PickIkPlugin | 59.7 % | 5125.9 / 6339.4 | 58.5 % | 5119.8 / 6574.2 | 9.9e-04 |

#### Unreachable poses (10 m away)

| Plugin | Wrongly solved | Median / p95 (us) |
|---|---|---|
| eaik_kinematics_plugin/EAIKKinematicsPlugin | 0.0 % | 4.7 / 4.9 |
| kdl_kinematics_plugin/KDLKinematicsPlugin | 0.0 % | 5126.2 / 5513.0 |
| cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin | 0.0 % | 32190.2 / 35811.6 |
| trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin | 0.0 % | 5142.3 / 5242.9 |
| pick_ik/PickIkPlugin | 0.0 % | 5478.6 / 6708.2 |

#### 5-joint group (base_link to link_5), reachable poses

| Plugin | Success | Median / p95 (us) | Max position error (m) | Max orientation error (rad) |
|---|---|---|---|---|
| eaik_kinematics_plugin/EAIKKinematicsPlugin | 100.0 % | 23.0 / 23.8 | 4.0e-13 | 3.8e-12 |
| kdl_kinematics_plugin/KDLKinematicsPlugin | 37.3 % | 5152.9 / 9181.3 | 9.7e-06 | 1.0e-05 |
| cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin | 88.3 % | 8201.6 / 46267.0 | 9.9e-06 | 9.8e-06 |
| trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin | 99.6 % | 5142.5 / 5249.5 | 1.6e-05 | 1.4e-05 |
| pick_ik/PickIkPlugin | 44.6 % | 5571.7 / 6072.1 | 1.0e-03 | 1.0e-03 |

Notes on the results:

- KDL, Cached KDL and TRAC-IK give one solution per pose, because they use the default
  multi-solution `getPositionIK()` of `KinematicsBase`. pick_ik returned no solution from the
  multi-solution `getPositionIK()` for any pose. The cause is not known.
- TRAC-IK uses its default solve type `Distance`. It searches for the full timeout for the solution
  nearest to the seed, so its time is always near 5 ms. The solve type `Speed` returns the first
  solution.
- pick_ik uses its default thresholds of 1e-3 m and 1e-3 rad, which explains its larger errors.
- Cached KDL returns a solution from its cache, which is often on a different branch from the seed. It
  can also take longer than the timeout. On the 5-joint group, 15 queries failed inside Cached KDL
  with the error "Seed state must have size 5 instead of size 6".
- EAIK fails on 4 % of the poses at the exact wrist singularity (joint_5 = 0).

## Licence

This repository is licensed under the BSD 3-Clause License, the same licence as EAIK and
MoveIt. See [LICENSE](LICENSE).

`eaik_vendor` downloads and builds third-party code with these licences:

| Project | Licence |
|---|---|
| [EAIK](https://github.com/OstermD/EAIK) | BSD 3-Clause, Copyright (c) 2025 Daniel Ostermeier |
| [ik-geo](https://github.com/OstermD/ik-geo) (EAIK submodule, fork of rpiRobotics/ik-geo) | BSD 3-Clause, Copyright (c) 2022 rpiRobotics |

`eaik_vendor` installs both licence texts to `share/eaik_vendor/licenses`.
