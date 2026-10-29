# inventory_inspection_poc

See the workspace [English README](../../README.md) for implementation details,
parameters, setup commands and the three recorded Nav2-only test results.
The [Chinese guide](../../README_FIRST_CN.md) contains additional setup history.

Current default: official Nav2 footprint approach protection enabled (2.0 s),
fixed stop/slowdown circles disabled. Launch with `pedestrian:=true` for this
mode; `extra_protection:=false` explicitly disables all monitor intervention.

Warehouse dimensions: 15 m x 15 m. Home (-5.5, 0, 0); inspection targets
A (-1.5, 1.5, pi/2), B (-1.5, -1.5, -pi/2), C (5.5, 0, pi).
The two horizontal shelves are 5 m x 1 m and the vertical shelf is 1 m x 5 m.

Official TurtleBot3 minimal simulation supplies the running gear, sensors and
bridge. At launch, the chassis is adapted to a 0.50 x 0.50 m box in both SDF
and URDF, and wheel track changes to 0.47 m. Installed assets are untouched.
Nav2 uses a local square footprint with 0.025 m padding and full MPPI footprint
checks; the NavFn global costmap uses a conservative 0.39 m enclosing circle.
Official Nav2 supplies AMCL, planning, control and WaitAtWaypoint.

`ros2 launch inventory_inspection_poc inspection.launch.py` brings up the world,
robot, navigation and mission. `run_mission:=false` permits single-goal testing.
Add `pedestrian:=true` to enable a HuNav worker-man with walking animation in the
central aisle, from (-4.0, 0) to (2.8, 0) and back at a nominal 0.6 m/s.
The default is disabled. Skin 4 and its textures are vendored from the same HuNav
revision referenced by the user's Summer-Research-2026 project; see models/hunav_worker/SOURCE.md.
Version 0.3.1 replaces static-body teleportation with an 8 N force-limited physical
slider, matching visible and collision geometry. Version 0.4.0 removes pedestrian
yielding: it has no robot pose, odometry or TF input and patrols independently.
Only stale actuator feedback causes a hold. Nav2's lidar costmaps, MPPI controller
and collision monitor own the robot's avoidance. Version 0.5.0 adds the skinned
visual following the physical body; collision geometry remains simplified, not per-bone.
Version 0.6.0 adds cosine acceleration/braking ramps, a stationary smooth turn,
per-physics-step visual tracking and distance-driven walking animation. Build both
packages with `colcon build --symlink-install --packages-up-to inventory_inspection_poc`.
Reproducible runtime tests and acceptance criteria are documented in
`../../docs/SMOOTH_PEDESTRIAN_TEST_CN.md`.
Route settings live in
`config/layout.yaml`. The complete path is checked against shelves and walls.
Nav2 has an optional historical slowdown/stop-zone configuration, a wider/faster local costmap and a
1.0 m/s forward limit in MPPI, velocity_smoother and Gazebo DiffDrive.
This is a limit; Nav2 can slow or stop. Detected contacts cancel the mission immediately.

The protected baseline is tagged `stable-simulation-with-extra-protection`.
On `experiment/nav2-only-navigation`, `extra_protection:=false` disables all
collision-monitor intervention while preserving the same MPPI, costmap, speed
and pedestrian parameters. The monitor remains a pass-through velocity relay;
the runtime probe verifies its live flags and compares input/output velocities.
Use `--extra-protection off` with `tools/verify_pedestrian_runtime.py` to test it.
Results and limitations are documented in `../../docs/NAV2_ONLY_EXPERIMENT_CN.md`.

The mission sends A/B/C to FollowWaypoints, verifies the action's terminal
status AND missed_waypoints, then uses NavigateToPose for Home (no extra dwell).
It records navigation results, TF/odometry trajectory, measured stationary dwell
and contact reports. Startup/action waiting is bounded. Results are written under
`~/inventory_poc_results`.

Reference implementations (installed externally; not vendored):

- https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation
- https://github.com/ros-navigation/navigation2
- https://docs.nav2.org/configuration/packages/nav2_waypoint_follower-plugins/wait_at_waypoint.html

Primary target: Ubuntu 26.04 / ROS 2 Lyrical / Gazebo Jetty, as specified in the
concept document. Jazzy/Harmonic uses its own installed Nav2 defaults and is
included as a migration target. Lyrical/Jetty is runtime tested locally;
Jazzy/Harmonic is not runtime tested. Humble/Fortress is excluded.
