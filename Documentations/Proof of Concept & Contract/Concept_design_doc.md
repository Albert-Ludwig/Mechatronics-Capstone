# Autonomous Inventory Inspection Robot

## Proof-of-Concept Technical Feasibility Report

**Project Name:** Autonomous Inventory Robot  
**Project Phase:** Proof of Concept (POC)  
**Primary Focus:** Nav2 integration and autonomous multi-waypoint navigation  
**Estimated Development Time:** One week  
**Report Status:** Preliminary technical proposal

---

## 1. Project Objective

This POC will evaluate the feasibility of an autonomous inventory inspection robot using ROS 2, Gazebo, and Nav2.

The robot will operate in a simulated warehouse. It will visit three inspection waypoints in a predefined order. At each waypoint, it will stop for five seconds to simulate image capture. It will then return to its starting pose and complete the mission.

The main goal is to integrate the Nav2 stack. Developing new navigation algorithms is outside the scope of this POC.

The POC will verify that:

1. The robot can localize itself on a pre-built map.
2. Nav2 can plan and execute a path to a goal pose.
3. The robot can visit multiple waypoints in a fixed order.
4. The robot can wait for five seconds at each inspection waypoint.
5. The robot can return to its starting pose autonomously.
6. The complete mission can run without human intervention.

---

## 2. System Specifications

| Item | Requirement |
|---|---|
| Operating system | Ubuntu 26.04 LTS |
| ROS distribution | ROS 2 Lyrical |
| Simulation platform | Gazebo Jetty |
| Navigation framework | Nav2 |
| Robot model | TurtleBot3 with differential-drive kinematics |
| Simulation environment | 15 m x 15 m indoor warehouse |
| Number of shelves | 3 |
| Shelf dimensions | 1 m x 5 m each |
| Shelf layout | Two horizontal shelves and one vertical shelf |
| Map | Pre-built 2D occupancy grid |
| Localization method | Adaptive Monte Carlo Localization (AMCL) |
| Inspection waypoints | Three fixed poses: A, B, and C |
| Dwell time | Five seconds at each waypoint |
| Starting pose | Fixed Home pose |
| Mission sequence | Home -> A -> B -> C -> Home |

### 2.1 Mission Sequence

```text
        System startup
              |
              v
    Localization initialization
              |
              v
            HOME
              |
              v
        Waypoint A
         Wait 5 s
              |
              v
        Waypoint B
         Wait 5 s
              |
              v
        Waypoint C
         Wait 5 s
              |
              v
        Return HOME
              |
              v
       Mission complete
```

After startup, the robot should complete the full sequence automatically. The operator should not need to send each navigation goal manually.

### 2.2 Out-of-Scope Functions

The following functions are excluded from this POC to limit development complexity:

- Physical image capture and image processing
- Product recognition and inventory counting
- Development of a SLAM mapping algorithm
- Custom path-planning or motion-control algorithms
- Optimization of waypoint order
- Dedicated dynamic-obstacle testing

These functions may be added during later stages of the Capstone project.

---

## 3. System Architecture

The POC contains three main subsystems:

1. Simulation environment
2. Localization and navigation
3. Waypoint mission execution

### 3.1 Simulation Environment

Gazebo will simulate a simplified 15 m x 15 m warehouse. The environment will contain:

- Four perimeter walls
- Three rectangular shelves
- One differential-drive TurtleBot3

Two shelves will be placed horizontally. The third shelf will be placed vertically. Simple rectangular models are sufficient. Detailed 3D modeling is not required.

Gazebo will simulate:

- Robot motion
- LiDAR measurements
- Wheel odometry
- Collisions between the robot and the environment

Nav2 also requires a 2D occupancy map that matches the Gazebo world.

**Main files:**

| File | Purpose |
|---|---|
| `warehouse.sdf` | Defines the warehouse, walls, shelves, and 3D simulation environment |
| `warehouse.pgm` | Stores the 2D occupancy grid |
| `warehouse.yaml` | Defines map metadata, including resolution and origin |

The Gazebo world and the Nav2 map must match. A mismatch can cause localization errors and navigation failure.

### 3.2 Localization and Navigation

The project will use existing Nav2 components. It will not develop low-level localization, path-planning, or motion-control algorithms.

```text
               Pre-built map
                     |
                     v
LiDAR -----------> AMCL <----------- Odometry
                     |
                     v
                Robot pose
                 (x, y, theta)
                     |
                     v
                    Nav2
                     |
           +---------+----------+
           |                    |
           v                    v
     Global planner       Local controller
           |                    |
           +---------+----------+
                     |
                     v
                  /cmd_vel
                     |
                     v
              Gazebo robot
```

#### AMCL Localization

AMCL uses the pre-built map, LiDAR scans, and odometry to estimate the robot's position and heading in the map frame:

$$
\mathbf{x} =
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
$$

Here, $x$ and $y$ describe the robot's position. $\theta$ describes its heading.

#### Global Planner

The global planner uses the current robot pose, goal pose, and map to compute a feasible path.

#### Local Controller

The local controller uses the global path and the robot's current state to compute velocity commands:

$$
\mathbf{u} =
\begin{bmatrix}
v \\
\omega
\end{bmatrix}
$$

where:

- $v$ is the linear velocity.
- $\omega$ is the angular velocity.

Nav2 sends velocity commands to Gazebo to drive the simulated robot.

### 3.3 ROS 2 Interfaces

The following ROS 2 topics must be configured:

| Topic | Purpose |
|---|---|
| `/map` | Pre-built 2D occupancy map |
| `/scan` | LiDAR scan data |
| `/odom` | Robot odometry |
| `/tf` | Coordinate transforms |
| `/cmd_vel` | Robot velocity commands |
| `/clock` | Simulation time |

The exact topic names and message types must be confirmed against the selected robot model and Nav2 configuration.

The system must provide the following transform chain:

```text
map -> odom -> base_link
```

- `map -> odom`: AMCL provides the global localization correction.
- `odom -> base_link`: The odometry system provides the robot's local motion estimate.

The `ros_gz_bridge` package will exchange the required data between Gazebo and ROS 2.

---

## 4. Waypoint Mission Execution

The POC will use the Nav2 Waypoint Follower for multi-waypoint navigation.

### 4.1 Waypoint Definition

Home and the three inspection waypoints will use fixed 2D poses:

$$
P_i = (x_i, y_i, \theta_i)
$$

where:

- $x_i$ and $y_i$ define the target position.
- $\theta_i$ defines the target heading.

Each inspection waypoint should be placed in front of its shelf. The robot should face the shelf. Exact coordinates will be selected after the warehouse layout is complete.

### 4.2 Autonomous Multi-Waypoint Navigation

The Waypoint Follower will execute this fixed sequence:

$$
Home \rightarrow A \rightarrow B \rightarrow C
$$

Nav2 will plan and control the robot's motion between consecutive waypoints.

### 4.3 Five-Second Dwell Time

The mission will use the Nav2 `WaitAtWaypoint` plugin.

```yaml
waypoint_pause_duration: 5000
```

The value is specified in milliseconds:

$$
5000\text{ ms} = 5\text{ s}
$$

After reaching a waypoint, the robot will wait for five seconds before continuing to the next waypoint.

### 4.4 Autonomous Return to Home

A simple Python mission script will coordinate the task. It will:

1. Wait for Nav2 and the localization system to become active.
2. Send the three inspection waypoints.
3. Check the mission result.
4. Send the Home goal after all inspection waypoints succeed.
5. Report the final navigation result as success or failure.

A complex mission-management system is not required for this POC.

---

## 5. Implementation Plan

The POC has five phases.

| Phase | Main Work | Acceptance Criterion |
|---|---|---|
| **Phase 1: Environment setup** | Configure ROS 2 Lyrical, Gazebo, and Nav2 | TurtleBot3 completes single-goal autonomous navigation |
| **Phase 2: Warehouse simulation** | Create the 15 m x 15 m warehouse and three shelves | Gazebo loads the world, and LiDAR detects the shelves |
| **Phase 3: Localization integration** | Create the 2D map and configure AMCL and TF | The robot localizes correctly on the map |
| **Phase 4: Mission integration** | Configure waypoints, the Waypoint Follower, and the five-second wait | The robot visits A, B, and C in order |
| **Phase 5: System verification** | Add the Home return and run repeated tests | The robot completes the full inspection mission autonomously |

### Recommended Development Sequence

First, complete single-goal navigation in the official Nav2 TurtleBot3 simulation.

Next, test multi-waypoint navigation and the wait behavior.

Finally, replace the example world with the custom warehouse and integrate all functions.

This sequence separates Nav2 configuration issues from custom-world issues. It should make debugging easier.

---

## 6. One-Week Development Schedule

| Day | Main Task | Expected Result |
|---|---|---|
| **Day 1** | Validate ROS 2, Gazebo, and Nav2 | The official TurtleBot3 simulation runs correctly |
| **Day 2** | Configure and test multi-waypoint navigation | A basic waypoint mission works |
| **Day 3** | Build the warehouse world and 2D map | The custom warehouse simulation is complete |
| **Day 4** | Integrate AMCL and Nav2 | The robot navigates in the custom warehouse |
| **Day 5** | Add the inspection mission, five-second waits, and Home return | The complete inspection sequence works |
| **Day 6** | Debug the system and repeat the tests | Mission reliability improves |
| **Day 7** | Perform final verification, prepare a demonstration, and complete the documentation | POC deliverables are ready |

The preliminary assessment is that the POC can be completed within one week, provided that:

- ROS 2 Lyrical and Gazebo run correctly.
- The official Nav2 TurtleBot3 simulation starts successfully.
- No low-level robot driver or navigation algorithm must be developed.
- No new features are added during the POC.

The main schedule risks are map alignment, TF configuration, AMCL, and Nav2 integration.

---

## 7. Team Responsibilities

Team responsibilities are to be assigned.

---

## 8. Verification and Acceptance Criteria

| Test | Acceptance Criterion |
|---|---|
| Localization | The robot maintains a valid pose estimate on the pre-built map |
| Single-goal navigation | The robot reaches a specified goal autonomously |
| Multi-waypoint navigation | The robot visits A -> B -> C in order |
| Inspection simulation | The robot remains stationary for about five seconds at each waypoint |
| Return Home | The robot returns to the predefined Home pose |
| Collision avoidance | No collision occurs during the mission |
| Autonomous operation | No operator input is required after mission startup |
| Repeatability | The robot completes three consecutive full missions successfully |

Also record:

- Total mission time
- Navigation success or failure
- Executed trajectory
- Waypoint completion status
- Failure cause, if any

Extensive performance testing is not required for this POC. The tests must show that the complete navigation mission runs reliably and repeatedly.

---

## 9. Feasibility and Risk Assessment

### 9.1 Technical Feasibility

The project uses established ROS 2 and Nav2 components:

- AMCL localization
- Global path planning
- Local motion control
- Waypoint Follower
- `WaitAtWaypoint`

The main work is system configuration and integration. It is not algorithm research. The existing navigation stack can provide all functions required by this POC.

### 9.2 Main Risks

| Risk | Potential Problem | Mitigation |
|---|---|---|
| ROS 2 Lyrical compatibility | Some tutorials or third-party plugins may not support this distribution | Use official Lyrical packages and examples first |
| Map and Gazebo mismatch | AMCL may estimate the wrong robot pose | Keep map dimensions, coordinates, and geometry consistent with the world |
| Incorrect TF or ROS interfaces | Nav2 may fail to navigate | Check LiDAR, odometry, TF, and velocity-control interfaces separately |
| Poor Nav2 parameter configuration | Navigation may fail or become unstable | Start with official parameters and adjust them step by step |
| Future physical-robot compatibility | Jetson or sensor drivers may not support the selected environment | Verify the hardware platform and ROS 2 distribution before deployment |

This POC does not need to resolve every physical deployment issue. The software architecture should remain as hardware-independent as possible.

---

## 10. Simulation-to-Real Robot Migration

The POC will use a simulated differential-drive TurtleBot3. The future physical robot is expected to use a similar differential-drive base. This will reduce migration effort.

### 10.1 Reusable Components

- Mission-execution logic
- Waypoint Follower
- Waypoint management
- Autonomous return-to-Home logic
- Nav2 architecture
- Standard ROS 2 communication interfaces

### 10.2 Components to Replace or Reconfigure

| Component | Required Change for Physical Deployment |
|---|---|
| Robot model | Replace it with the physical robot's URDF |
| LiDAR | Connect the real sensor and its driver |
| Odometry | Use real wheel encoders and fuse IMU data if needed |
| Motor control | Replace the Gazebo control interface with the physical motor-control interface |
| AMCL | Reconfigure it for the real sensors and environment |
| Nav2 controller | Tune it for the physical base's velocity, acceleration, and motion behavior |
| Robot footprint | Update the costmap footprint to match the physical robot |
| Map | Create a map of the deployment environment |

The upper-level navigation and mission logic should remain unchanged where possible. Most migration work should involve hardware interfaces and robot-specific parameters.

Confirm compatibility between ROS 2 Lyrical and the final onboard computer early. If the team selects an NVIDIA Jetson, confirm compatibility with its exact model, JetPack version, and device drivers.

---

## 11. Expected Deliverables

| Deliverable | Description |
|---|---|
| Warehouse simulation | Gazebo world with a 15 m x 15 m warehouse and three shelves |
| Occupancy map | 2D map aligned with the simulation world |
| Nav2 configuration | Working AMCL, planner, and controller configuration |
| Waypoint mission | Three inspection stops, a five-second wait at each stop, and an autonomous Home return |
| Verification results | System test results and repeated-run results |
| Demonstration video | Video of the complete autonomous inspection mission |
| Technical documentation | Software environment, configuration, and operating instructions |

---

## 12. Conclusion

This POC will use ROS 2 Lyrical, Gazebo, and Nav2 to perform autonomous multi-waypoint inventory inspection in a simplified warehouse simulation.

The project uses existing Nav2 localization, path-planning, motion-control, and mission-execution components. It does not require a custom low-level navigation algorithm.

The preliminary assessment is that the POC can be completed within one week, provided that the software environment works and the project scope remains fixed.

A successful POC will demonstrate that:

1. The robot can localize itself on a pre-built map.
2. Nav2 can plan and control motion between goal poses.
3. The robot can visit three inspection waypoints in a fixed order.
4. The robot can wait for five seconds at each waypoint.
5. The robot can return to its starting pose autonomously.
6. The complete mission can run reliably and repeatedly.

The POC will provide a foundation for navigation-system development and integration on the future physical robot.

---

## Appendix: Items to Confirm Before Development

### A. Warehouse and Waypoint Coordinates

The following requirements are defined:

- Warehouse dimensions: 15 m x 15 m
- Number of shelves: Three
- Shelf dimensions: 1 m x 5 m each
- Shelf layout: Two horizontal shelves and one vertical shelf
- Number of inspection waypoints: Three
- Starting point: Fixed Home pose

The exact shelf and waypoint coordinates still need to be defined. They must be consistent across the Gazebo world, 2D map, and mission configuration.

### B. Robot Model

The POC will use the official Nav2 TurtleBot3 differential-drive simulation model.

A custom robot URDF will not be developed during the POC. The model will be replaced during physical-robot integration.

### C. Software Environment

The current proposal assumes:

- Ubuntu 26.04 LTS
- ROS 2 Lyrical
- Gazebo Jetty
- Nav2

Confirm the installed versions before development. Also verify that the official Nav2 TurtleBot3 example starts and runs correctly.

### D. Physical Robot Compatibility

After selecting the onboard computer, LiDAR, IMU, and mobile base, confirm:

- ROS 2 distribution support
- Sensor-driver compatibility
- Motor-control interface
- Sources of TF and odometry data
- Compatibility between the onboard computer and operating system

These items are outside the direct acceptance scope of this POC. They remain technical constraints for later physical-robot deployment.
