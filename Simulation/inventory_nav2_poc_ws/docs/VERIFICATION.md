# Delivery verification

This file records checks on the implementation, not a simulated mission result.

## Passed

- 20 Python unittest cases: generated assets, world/map collision geometry,
  shelf dimensions and orientations, goal-facing direction, conservative 0.55 m
  clearance connectivity, parameter merge, Twist/TwistStamped selection,
  terminal action failures, missed-waypoint failures, dwell measurement,
  telemetry gaps, mission order, abort-before-Home and bounded startup waiting.
- Regression check for the observed Lyrical startup crash: project overrides
  must not emit untyped `filters: []`. Inherited filter lists are preserved;
  the launch disables keepout/speed zones through the upstream launch switches.
- Python source compilation and setuptools metadata check.
- Source contract checks against both official Jazzy and Lyrical branches.
- Workspace layout image rendered from the actual layout and visually inspected.
- 0.50 x 0.50 m chassis, 0.47 m wheel track, square local footprint and global
  enclosing-circle checks; C behind the far shelf at (5.5, 0, pi).
- Both upstream model versions were actually transformed and parsed in the
  source-contract test; resized SDF/URDF dimensions and wheel joints agree.

## Official source versions inspected

| Distribution | navigation2 commit | minimal TurtleBot3 commit | cmd_vel |
|---|---|---|---|
| Jazzy | `645abd95f2be02a13ca539b29c2ddc065db34f89` | `b9d523ad7ea0e98174627fdefeb4b1ae9b515063` | Twist |
| Lyrical | `4e5b057d7d9d74a14155693a9eab6f4c0c2a162f` | `1eeeae4f4d2b8af3abcb93b972d959a3a9b50ff7` | TwistStamped |

The launch uses the installed distribution's defaults and bridge, rather than
shipping either reference checkout. Users may have newer binary package releases;
the read-only preflight checks required assets and action types on their machine.

## Not run here

The execution environment is Ubuntu 24.04 without ROS/Gazebo. The ROS package
server request returned HTTP 502, so complete dynamic verification was unavailable.

The following remain on-machine acceptance work:

1. Successful ROS dependency installation and preflight.
2. Gazebo spawning, LiDAR, odometry, clock and TF reception.
3. AMCL localization and single-goal Nav2 navigation.
4. Full A/B/C inspection mission with measured five-second pauses.
5. Return Home and collision-event monitoring.
6. Three consecutive successful runs, with each run starting a fresh simulation.
7. Demonstration recording.

No `mission_report.json`, trajectory or demonstration video has been fabricated.

## Reproduce local code checks

From `src/inventory_inspection_poc`:

```bash
python3 tools/generate_assets.py
python3 -m unittest discover -s tests -v
python3 -m compileall -q inventory_inspection_poc launch tools
python3 setup.py check
```
