# Official Nav2 approach protection

Updated on 2026-10-28 (America/Toronto). Branch:
`simulation/nav2-approach-protection`.

## Requested behavior

Use official Nav2 navigation and protection components, keeping custom code for
simulation, inspection-goal orchestration and validation. Restore the official
Collision Monitor `FootprintApproach` check while keeping the two fixed circles
disabled. The fixed circles are also official Nav2 functionality; their use and
dimensions were project configuration, rather than custom control algorithms.

## Configuration

| Setting | Current value |
| --- | --- |
| `collision_monitor.enabled` | `true` |
| `FootprintApproach.enabled` | `true` |
| `FootprintApproach.time_before_collision` | `2.0 s` |
| `FootprintApproach.min_points` | `2` |
| `StopCircle.enabled` / `SlowCircle.enabled` | `false` / `false` |
| Circle visualization | Disabled |
| Collision Monitor `scan.enabled` | `true` |
| MPPI / costmap / velocity smoother / robot / pedestrian parameters | Same as the saved pass-through experiment |

`FootprintApproach` uses the official `approach` action and the local costmap's
published footprint, inherited from the installed Nav2 defaults. It can reduce
the smoothed velocity command based on forward collision simulation. It does
not forecast the pedestrian's future decisions. MPPI's own footprint collision
checks and trajectory validator remain active.

The launch argument `extra_protection` is retained for compatibility. Its default
`true` now enables approach protection with no fixed circles. Explicitly setting
it to `false` disables all monitor intervention for the historical experiment.
The saved Git tags retain their original configurations.

The custom mission node still records actual contacts, publishes an emergency
zero and cancels navigation after contact. This is test termination after a
failed run, not an additional preventive robot avoidance policy. The mission
node normally sends Nav2 action goals. The pedestrian follows its own motion
profile and does not yield based on robot position.

## Run and test

```bash
./run_poc.sh pedestrian:=true
```

After building and sourcing ROS 2 Lyrical and this workspace, run an isolated
fresh-world acceptance test with:

```bash
python3 src/inventory_inspection_poc/tools/verify_pedestrian_runtime.py \
  --mode mission --extra-protection on --timeout 360
```

The observer verifies all five live monitor switches, requires approach and scan
to be enabled, and rejects either fixed circle being enabled or reporting an
action. It also checks full mission acceptance, contacts, physical stability,
pedestrian bounds, Nav2 commands and dynamic costmap observations. Velocity
differences are recorded but are allowed when official protection intervenes.

The desktop validation uses `ROS_DOMAIN_ID=44` and
`GZ_PARTITION=inventory_nav2_visible`; observing that running service requires the
same values. These settings isolate communication and do not change control.

## Validation

All 33 unit tests passed, including default approach-on/circles-off configuration
and all-monitor-off parameter isolation checks. Both packages built successfully;
Python compilation and whitespace checks passed. Non-monitor overrides were
compared directly with `simulation-without-extra-protection` and were unchanged.

One fresh-world desktop mission passed automated acceptance. Raw evidence:
[nav2-approach-run-1.json](runtime/nav2-approach-run-1.json).

| Measurement | Result |
| --- | --- |
| Route | Home → A → B → C → Home; successful |
| Mission simulation / mission-node wall time | 75.024 / 81.959 s |
| Measured dwell at A / B / C | 5.372 / 5.202 / 5.406 s |
| Detected contacts / contact publisher coverage | 0 / 8 of 8 |
| Final Home distance / yaw error | 0.128 m / 0.169 rad |
| Minimum sampled robot–pedestrian center distance | 0.882 m; not surface clearance |
| Dynamic costmap observations | 34 |
| Observed maximum robot / pedestrian speed | Approximately 1.0 / 0.6 m/s |
| Maximum robot model height / tilt | 0.010007 m / 4.035e-5 rad |
| Live monitor / footprint approach / scan flags | All true |
| Live stop / slowdown circle flags | Both false |
| Fixed-circle state messages | None |
| `FootprintApproach` APPROACH state messages | 2 |
| Velocity sample pairs / moving pairs | 1173 / 1112 |
| `acceptance_passed` | `true` |

Input/output velocity differences were nonzero, consistent with the active
official protection changing commands. The state-message count records reported
actions, not the total number of modified commands or braking duration.
The original observer evidence directory is `/tmp/nav2-mission-p09kskpy`.
The result JSON is copied into Git so the summary does not depend on that
temporary directory.

This is one full runtime regression check for the restored approach protection;
the three earlier pass-through runs remain evidence for the monitor-off mode.
The desktop scene is left running with approach protection enabled and the robot
returned Home. No demonstration video was recorded for this run.
