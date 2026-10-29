# Warehouse Navigation Simulation with Nav2

This workspace runs a warehouse inspection mission with an independently moving
pedestrian. Nav2 performs localization, path planning and robot control. The
robot visits **Home → A → B → C → Home**, stopping for five seconds at each
inspection point.

The current default enables official Nav2 `FootprintApproach` collision-time
protection with a 2.0 s threshold. The fixed stop and slowdown circles are
disabled. Normal preventive avoidance uses Nav2 components; custom Python code
provides mission orchestration, simulation and test observation.

This workspace also preserves a tested mode without Collision Monitor intervention.
Three fresh-world missions passed automated acceptance with no detected contact.
The robot's maximum forward speed is **1.0 m/s** and the pedestrian's nominal
cruising speed is **0.6 m/s**. Robot speed is chosen by Nav2 within its limits.

## Saved versions

| Git reference | Contents |
| --- | --- |
| `stable-simulation-with-extra-protection` | Protected baseline at `002773a`; stop circle, slowdown circle and footprint approach protection enabled. |
| `simulation-without-extra-protection` | Historical snapshot at `5ca68a9`, with all preventive monitor intervention disabled when explicitly requested. |
| `experiment/nav2-only-navigation` | Branch containing the experiment and documentation; experiment implementation and runtime evidence were committed in `141d811`. |
| `simulation/nav2-approach-protection` | Current configuration: official footprint approach protection enabled, fixed circles disabled. |

**The launch default remains `extra_protection:=true`.** To reproduce the
unprotected experiment, explicitly pass `pedestrian:=true extra_protection:=false`.
The Git tag identifies the snapshot; it does not change launch defaults.
In the current configuration, `true` enables footprint approach protection only;
it does not re-enable the fixed circles. The protected baseline tag preserves the
older configuration with all three checks enabled.
Current-mode validation is documented in
[Nav2 approach protection](docs/NAV2_APPROACH_PROTECTION.md).
One fresh-world mission with approach protection restored passed in 75.024 s of
simulation time with zero detected contacts; the live flags and protection state
messages confirmed that the official check was active.

## What was implemented

- A 15 m × 15 m Gazebo warehouse, three shelves, a static localization map and
  inspection goals generated from a shared layout configuration.
- An adaptation of the installed official TurtleBot3 Waffle model to a
  0.50 m × 0.50 m chassis. SDF/URDF dimensions, wheel spacing, inertia, physical
  collision geometry and Nav2 footprints are kept consistent. Installed ROS
  assets are read at launch and are not edited.
- Official Nav2 AMCL localization, NavFn global planning, MPPI local control,
  lidar-based costmaps, velocity smoothing and waypoint dwell behavior.
  Project-specific parameters configure these components; there is no custom
  robot avoidance controller.
- An independent pedestrian moving between the aisle opening and the front of
  shelf C. It uses smooth acceleration/braking, stationary turns and a HuNav
  worker-man skin with distance-driven walking animation. A custom Gazebo visual
  plugin follows its physical body on each physics step. Simplified collision
  geometry is separate from the animated skin.
- A physical pedestrian slider with bounded travel and actuator force, replacing
  static-collider teleportation. The pedestrian reads its own joint feedback,
  not robot pose, odometry or TF. It does not yield to the robot. Stale joint
  feedback causes it to hold until feedback resumes.
- A mission node that waits for navigation, localization and pedestrian readiness,
  initializes at Home, submits inspection goals through `FollowWaypoints`, checks
  for missed waypoints, returns via `NavigateToPose`, and writes reports and a
  trajectory. It does not prescribe the path around shelf C.
- Contact sensors, measured stationary dwell, physical-state checks and an
  isolated runtime verification tool. Actual contact causes the mission to
  publish zero velocity, cancel navigation and fail the test.

## Control chain and extra protection

The normal velocity chain is:

```text
Nav2 MPPI → /cmd_vel_nav
          → Nav2 velocity_smoother → /cmd_vel_smoothed
          → Nav2 Collision Monitor → /cmd_vel → Gazebo DiffDrive
```

With `extra_protection:=false`, the Collision Monitor process stays in the
official launch chain as a velocity relay. Its global switch, all configured
polygons and its observation sources are disabled. This also prevents a missing
monitor input from causing a source-timeout stop. Lidar input to the Nav2 costmaps
remains enabled.

| Additional monitor behavior | Historical protected baseline | Current default | Pass-through experiment |
| --- | --- | --- | --- |
| `StopCircle` | Radius 0.60 m; stop; minimum 2 points | Disabled | Disabled |
| `SlowCircle` | Radius 1.15 m; speed ratio 0.5; minimum 2 points | Disabled | Disabled |
| `FootprintApproach` | Collision-time threshold 2.0 s; minimum 2 points | Enabled, same threshold | Disabled |
| Monitor global switch and `scan` source | Enabled | Enabled | Disabled |

All three behaviors are supplied by official Nav2 Collision Monitor. Their
selection and numerical settings are project configuration, not custom avoidance
implementations. Approach protection is independent of MPPI and may reduce its
smoothed command based on forward collision simulation against observed points.

Circle radii are measured from the robot base frame, not from the chassis edge.
`FootprintApproach` is an additional Collision Monitor check; MPPI's own
trajectory collision checking remains active in the experiment.

Velocity smoothing, Gazebo speed/acceleration limits, and the contact-triggered
failure handler also remain active. Contact handling occurs after detected
contact and cannot turn a collision into a successful avoidance result. Thus,
"without extra protection" means without preventive Collision Monitor
intervention, while retaining the configured Nav2 and physical drive chain.

## Parameters used in the recorded experiment

Overrides are merged with the **installed distribution's** `nav2_bringup`
defaults. The tables below give the principal settings, not a replacement for
the complete effective configuration. The launch log prints the generated
parameter-file path. The installed Lyrical defaults supply NavFn, MPPI's
`diff_drive` motion model and `model_dt=0.05 s`.

### Robot, navigation and costmaps

| Parameter | Value |
| --- | --- |
| Chassis length × width × height | 0.50 × 0.50 × 0.089 m |
| Configured chassis mass / wheel separation | 1.0 kg / 0.47 m |
| Forward / reverse linear velocity limits | +1.0 / −0.30 m/s |
| Angular velocity limit | ±1.2 rad/s |
| Linear acceleration / braking magnitude | 1.0 / 2.0 m/s² |
| Angular acceleration magnitude | 2.0 rad/s² |
| MPPI controller frequency | 20 Hz |
| MPPI prediction steps / batch size | 60 / 1000 |
| Prediction step / nominal horizon | 0.05 s / 3.0 s |
| Sampling standard deviations `vx_std` / `wz_std` | 0.35 m/s / 0.6 rad/s |
| `CostCritic` | Full footprint; weight 10.0; near-goal distance 0.4 m |
| `GoalAngleCritic.threshold_to_consider` | 0.18 m |
| Trajectory validator | Full footprint; collision lookahead 2.5 s |
| Goal position / yaw tolerance | 0.18 m / 0.20 rad |
| Progress checker | Movement radius 0.25 m; allowance 30 s |
| Local costmap size / resolution | 8 × 8 m / 0.05 m |
| Local costmap update / publication | 20 / 5 Hz |
| Local footprint | Square with corners `(±0.25, ±0.25)` m; padding 0.025 m |
| Global costmap update / resolution | 5 Hz / 0.05 m |
| Global enclosing robot radius | 0.39 m |
| Local and global inflation radius | 1.1 m |
| Lidar obstacle marking / ray clearing range | 5.0 / 6.0 m |
| AMCL particles / motion update thresholds | 500–1500 / 0.10 m and 0.10 rad |
| Velocity smoother | Maximum `[1.0, 0.0, 1.2]`; minimum `[-0.30, 0.0, -1.2]`; acceleration and braking limits as above |

Costmap inflation is a Nav2 obstacle-cost field, rather than a fixed external
stop circle. MPPI and the costmaps were identical between the protected baseline
and the recorded unprotected experiment; no controller retuning was needed for
these three successful runs.

### Pedestrian and mission

| Parameter | Value |
| --- | --- |
| Pedestrian path | `x=-4.0…2.8 m`, `y=0.0 m`, back and forth |
| Nominal cruising speed | 0.6 m/s |
| Acceleration / braking ramp | 2.0 s cosine ramp each |
| Stationary endpoint turn interval | 3.2 s |
| Visual heading-rate limit | 1.4 rad/s |
| Animation scale | 4.15 animation seconds per metre |
| Configured path radius / obstacle clearance | 0.28 / 0.20 m; path validation, not a robot braking zone |
| Physical pedestrian mass / actuator-force limit | 10 kg / 8 N |
| Home `(x, y, yaw)` | `(-5.5, 0.0, 0.0)` |
| A `(x, y, yaw)` | `(-1.5, 1.5, π/2)` |
| B `(x, y, yaw)` | `(-1.5, -1.5, -π/2)` |
| C `(x, y, yaw)` | `(5.5, 0.0, π)`; behind shelf C |
| Dwell at A / B / C | 5 s each, using Nav2 `WaitAtWaypoint` |
| Startup / mission timeout | 240 / 900 s |

## Build and run

The recorded runtime tests used **ROS 2 Lyrical and Gazebo Sim 10 (Jetty)** on
the local Ubuntu 26.04 installation. The launcher also accepts Jazzy, but this
snapshot's runtime results apply to Lyrical. Required dependencies include Nav2,
`nav2_minimal_tb3_sim`, ROS–Gazebo bridges, RViz, colcon, Python YAML and the
Gazebo development/vendor packages declared in both package manifests.

From a shell with these dependencies installed:

```bash
cd /home/qhl/inventory_nav2_poc_ws
source /opt/ros/lyrical/setup.bash
colcon build --base-paths src --symlink-install --packages-up-to inventory_inspection_poc
source install/local_setup.bash
ros2 run inventory_inspection_poc check_environment
ros2 launch inventory_inspection_poc inspection.launch.py \
  pedestrian:=true extra_protection:=true
```

The convenience launcher builds, sources, checks and launches the same scene:

```bash
./run_poc.sh pedestrian:=true
```

The launcher limits package discovery to this workspace's `src` directory so a
nested repository copy cannot cause duplicate ROS package names. In VS Code
Snap terminals it also ignores Snap-derived `GTK_PATH` and `GIO_MODULE_DIR`
values for its child processes. Those module paths can load an incompatible
Snap `libpthread` into system RViz and cause a `GLIBC_PRIVATE` symbol error.
The official RViz launch shuts down the whole scene when RViz exits, so that
initial library failure can produce secondary ROS shutdown crash reports.
Non-Snap module paths are preserved, and the parent terminal is unaffected.
The [original loader error](docs/runtime/snap-rviz-library-failure.log) was
reproduced directly. A full desktop run with both problematic Snap paths
explicitly injected then passed after the fix: 74.252 s of mission simulation
time, zero detected contacts, and approach protection enabled. See the
[regression evidence](docs/runtime/nav2-snap-startup-regression.json).

Set `run_mission:=false` for manual goals, or
`headless:=true use_rviz:=false` to hide both interfaces. Headless operation still
requires a working rendering environment for GPU lidar. Exit the complete scene
before launching another in the same ROS domain and Gazebo partition. Restart
the entire launch for a fresh mission from Home.

Use `extra_protection:=false` for the historical pass-through comparison. The
pedestrian is optional and defaults to `pedestrian:=false`.

Normal mission outputs are timestamped beneath `~/inventory_poc_results/`:
`mission_report.json` contains outcomes and acceptance measurements, and
`trajectory.csv` records the observed trajectory.

## Test method and reproduction

After building and sourcing the workspace, run the ROS-independent unit checks:

```bash
python3 -m unittest discover -s src/inventory_inspection_poc/tests -v
```

For one isolated full mission without extra protection:

```bash
python3 src/inventory_inspection_poc/tools/verify_pedestrian_runtime.py \
  --mode mission --extra-protection off --timeout 360
```

To test the **current default** with official footprint approach protection, use
`--extra-protection on` instead. The observer then requires live flags
`enabled=true`, `FootprintApproach.enabled=true`, `scan.enabled=true`, and both
circle flags false. Velocity differences are expected when Nav2 protection acts;
the zero-difference assertion applies only to the `off` experiment.

Run this command three separate times to repeat the fresh-world experiment.
Each invocation launches a new world in an isolated ROS domain and Gazebo
partition, prints its `/tmp/nav2-mission-...` evidence directory, and cleans up
its own processes. Evidence includes `verification.json`, launch logs and a
mission report/trajectory under `results/`. Save desired evidence before
temporary files are removed by the operating system.

The mission and observer together check:

1. Successful A/B/C actions without missed waypoints, and successful return Home.
2. Measured stationary dwell: at least 4.5 s per point for a configured 5 s pause.
   Valid samples must be within 0.23 m / 0.25 rad of the active target, with
   speeds at most 0.02 m/s / 0.04 rad/s and no observation gap above 0.6 s.
3. Final Home error at most 0.25 m and 0.30 rad.
4. All eight expected wall/shelf/pedestrian contact publishers registered, and
   no detected contact. Sensors are event-only; publisher discovery alone is not
   an independent proof of the entire contact-detection chain.
5. Pedestrian feedback, motion bounds, visual tracking, robot physical stability,
   Nav2 velocity commands and pedestrian observations in the dynamic costmap.
6. Five live monitor flags all false: `enabled`, `StopCircle.enabled`,
   `SlowCircle.enabled`, `FootprintApproach.enabled` and `scan.enabled`.
7. Timestamp-matched `/cmd_vel_smoothed` and `/cmd_vel` samples, comparing
   forward, lateral and angular velocity. The maximum difference must be below
   `1e-9`, there must be more than ten moving pairs, and no monitor actions.

If contact occurs, the run fails. Pass-through comparisons exclude the last
0.1 s before the reported contact and subsequent samples so that failure-handler
zero commands are not confused with preventive intervention. No such exclusion
was necessary in the three successful runs.

## Historical pass-through results

These results apply to `extra_protection:=false`, not the subsequently restored
approach protection. Recorded on **2026-10-28 (America/Toronto)**. All three missions started in newly
created worlds. Runs 1 and 2 were headless; run 3 used the desktop scene with an
attached observer. These are automated results; three demonstration videos and
three complete manual visual inspections were not recorded.

| Run / raw evidence | Mission simulation time | Total mission-node wall time | Detected contacts | Minimum sampled center distance | Final Home distance / yaw error | Acceptance |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| [1](docs/runtime/nav2-only-run-1.json) | 73.628 s | 79.745 s | 0 | 0.956 m | 0.033 m / 0.157 rad | Pass |
| [2](docs/runtime/nav2-only-run-2.json) | 73.212 s | 78.976 s | 0 | 1.120 m | 0.084 m / 0.177 rad | Pass |
| [3](docs/runtime/nav2-only-run-3.json) | 74.702 s | 81.172 s | 0 | 0.544 m | 0.076 m / 0.171 rad | Pass |

Minimum center distance is sampled between robot and pedestrian centers; it is
not surface clearance. Wall time includes mission-node startup and waiting,
while simulation time is measured from the mission's start marker.

| Verification measurement | Run 1 | Run 2 | Run 3 |
| --- | ---: | ---: | ---: |
| Measured dwell at A / B / C (s) | 5.202 / 5.406 / 5.372 | 5.202 / 5.406 / 5.406 | 5.406 / 5.202 / 5.406 |
| Contact publisher coverage | 8/8 | 8/8 | 8/8 |
| Dynamic-costmap observations | 36 | 33 | 33 |
| Paired velocities / moving pairs | 1148 / 1085 | 1139 / 1077 | 1168 / 1106 |
| Maximum paired velocity difference | 0.0 | 0.0 | 0.0 |
| Collision Monitor actions | 0 | 0 | 0 |

All five live protection flags were false in each run. Maximum observed robot
speed was approximately 1.0 m/s and pedestrian joint speed was 0.6 m/s, with
floating-point differences around `1e-12`. Maximum robot model height was about
0.010007 m and tilt about `4.035e-5 rad`; no launching or tipping was observed
in the sampled physical state.

The implementation also passed **32 unit tests**, a colcon build and Python
compilation checks. A dedicated configuration test verifies that disabling
protection changes only Collision Monitor parameters. The actual installed
default/override merges were also compared: layout and all non-monitor
navigation parameters matched the protected baseline.

Two additional attempts failed during startup service discovery before any
navigation. They are recorded separately and are not counted as navigation
passes: [action discovery timeout](docs/runtime/nav2-only-startup-failure.log),
[costmap service discovery timeout](docs/runtime/nav2-only-desktop-startup-failure.log)
and its [observer report](docs/runtime/nav2-only-desktop-startup-failure.json).
The successful desktop retry used `ROS_DOMAIN_ID=44` and
`GZ_PARTITION=inventory_nav2_visible`; this isolated communication without
changing navigation parameters.

For this scene and parameter set, Nav2 completed three full inspection missions
without preventive Collision Monitor intervention and without detected contact.
These runs do not cover arbitrary crossing phases, sudden intrusions or multiple
pedestrians, and do not establish a general collision-free guarantee.

## Source and evidence map

For C++ IntelliSense, the visual plugin's CMake build exports
`build/inventory_pedestrian_visual/compile_commands.json`. Set `compileCommands`
in your VS Code C/C++ configuration to
`${workspaceFolder}/build/inventory_pedestrian_visual/compile_commands.json`.
This supplies the versioned Gazebo vendor include paths and definitions used by
the actual compiler; `/opt/ros/lyrical/include` alone does not contain those
headers. Build the package once before opening its C++ source.

| File | Purpose |
| --- | --- |
| [layout.yaml](src/inventory_inspection_poc/config/layout.yaml) | World, robot, pedestrian and mission settings |
| [nav2_overrides.yaml](src/inventory_inspection_poc/config/nav2_overrides.yaml) | Nav2 parameter overrides |
| [configuration.py](src/inventory_inspection_poc/inventory_inspection_poc/configuration.py) | Parameter merging, derived footprints and protection switch |
| [inspection.launch.py](src/inventory_inspection_poc/launch/inspection.launch.py) | Simulation/navigation launch |
| [mission.py](src/inventory_inspection_poc/inventory_inspection_poc/mission.py) | Goal submission, readiness, contact abort and reporting |
| [acceptance.py](src/inventory_inspection_poc/inventory_inspection_poc/acceptance.py) | Action-result and stationary-dwell checks |
| [robot_model.py](src/inventory_inspection_poc/inventory_inspection_poc/robot_model.py) | Physical/visual chassis and drive adaptation |
| [pedestrian.py](src/inventory_inspection_poc/inventory_inspection_poc/pedestrian.py) | Pedestrian path, physical model and motion profile |
| [pedestrian_motion.py](src/inventory_inspection_poc/inventory_inspection_poc/pedestrian_motion.py) | Independent pedestrian actuator commands |
| [smooth_actor.cc](src/inventory_pedestrian_visual/src/smooth_actor.cc) | Smooth visual following and animation |
| [verify_pedestrian_runtime.py](src/inventory_inspection_poc/tools/verify_pedestrian_runtime.py) | Runtime observer and isolated test launcher |
| [HuNav asset provenance](src/inventory_inspection_poc/models/hunav_worker/SOURCE.md) | Model origin, pinned revisions and license information |
| [Chinese experiment record](docs/NAV2_ONLY_EXPERIMENT_CN.md) | Detailed experiment notes |

Inspection dwell simulates a capture stop; this workspace does not implement
inventory recognition or save inspection camera images.
