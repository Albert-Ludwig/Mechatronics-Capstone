# HuNav worker-man visual assets

Selected to match skin 4 in Albert-Ludwig/Summer-Research-2026, revision
326ed17307f9c5b748c59a37538c52e260b8ea7a, scenario
Humble_Migration_20260729/pipeline_source/experiment_setup/hunav/scenarios/office_2_agents.yaml.

Source: https://github.com/robotics-upo/hunav_gazebo_fortress_wrapper
Pinned revision: 41f97f764df00303a85227911ec8173f0c25c758
Files: worlds/models/worker_man.dae, walk.dae and the six image files referenced
by worker_man.dae. These are unchanged upstream files; see source-sha256.json.

The upstream MIT license is included as LICENSE. The worker skin and walking
animation are the same asset family used by the referenced HuNav installation.
Only visual assets are reused; no HuNav agent controller or robot avoidance
code is imported. Gazebo FollowActor animates the skin following the physical
pedestrian model. The physical model remains an independent bounded slider.
