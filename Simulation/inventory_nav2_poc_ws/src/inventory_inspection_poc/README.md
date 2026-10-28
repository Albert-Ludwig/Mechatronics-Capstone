# inventory_inspection_poc

See `../../README_FIRST_CN.md` in the source workspace for complete setup and validation.

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
included as a migration target; neither runtime combination has been executed
in the delivery environment. Humble/Fortress is excluded.
