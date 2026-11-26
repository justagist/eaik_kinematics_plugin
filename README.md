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

`benchmark_eaik_kinematics` compares this plugin with `kdl_kinematics_plugin` on the Fanuc arm. It
uses the forward kinematics of random configurations as reachable poses, and moves each pose 10 m
away to get unreachable poses. Each call has a random seed and a timeout of 5 ms.

```bash
ros2 run eaik_kinematics_plugin benchmark_eaik_kinematics 1000
```

Results on an Intel Core i7-14650HX, ROS 2 Jazzy, Release build, 1000 poses:

| Plugin | Reachable: success | Reachable: median / p95 (us) | Unreachable: median / p95 (us) | getPositionIKs: median solutions |
|---|---|---|---|---|
| eaik_kinematics_plugin/EAIKKinematicsPlugin | 100.0 % | 15.9 / 23.5 | 5.3 / 7.8 | 16 (min 4, max 48) |
| kdl_kinematics_plugin/KDLKinematicsPlugin | 70.0 % | 1772.6 / 7645.5 | 5127.7 / 5505.1 | 1 (min 1, max 1) |

KDL gives one solution per pose, because it uses the default multi-solution `getPositionIK()` of
`KinematicsBase`.

## Licence

This repository is licensed under the BSD 3-Clause License, the same licence as EAIK and
MoveIt. See [LICENSE](LICENSE).

`eaik_vendor` downloads and builds third-party code with these licences:

| Project | Licence |
|---|---|
| [EAIK](https://github.com/OstermD/EAIK) | BSD 3-Clause, Copyright (c) 2025 Daniel Ostermeier |
| [ik-geo](https://github.com/OstermD/ik-geo) (EAIK submodule, fork of rpiRobotics/ik-geo) | BSD 3-Clause, Copyright (c) 2022 rpiRobotics |

`eaik_vendor` installs both licence texts to `share/eaik_vendor/licenses`.
