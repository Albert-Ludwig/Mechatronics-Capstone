#!/usr/bin/env python3
"""Check our overrides against local official Nav2/TB3 git reference checkouts.

Developer check only; users do not need to clone the references to run the POC.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import yaml
from inventory_inspection_poc.configuration import (
    detect_stamped_velocity, load_layout, make_nav2_parameters)
from inventory_inspection_poc.robot_model import adapt_sdf, adapt_urdf


def git_text(repo, revision, path):
    return subprocess.check_output(['git', '-C', str(repo), 'show', revision+':'+path], text=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--nav2', type=Path, required=True)
    parser.add_argument('--tb3', type=Path, required=True)
    args = parser.parse_args()
    references = {
        'jazzy': ('645abd95f2be02a13ca539b29c2ddc065db34f89',
                  'b9d523ad7ea0e98174627fdefeb4b1ae9b515063'),
        'lyrical': ('4e5b057d7d9d74a14155693a9eab6f4c0c2a162f',
                    '1eeeae4f4d2b8af3abcb93b972d959a3a9b50ff7'),
    }
    overrides = yaml.safe_load((ROOT/'config/nav2_overrides.yaml').read_text())
    layout = load_layout(ROOT/'config/layout.yaml')
    reports = []
    for distro, (nav_revision, tb_revision) in references.items():
        defaults = yaml.safe_load(git_text(args.nav2, nav_revision,
                                           'nav2_bringup/params/nav2_params.yaml'))
        bridge = git_text(args.tb3, tb_revision,
                          'nav2_minimal_tb3_sim/configs/turtlebot3_waffle_bridge.yaml')
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'bridge.yaml'
            path.write_text(bridge)
            stamped = detect_stamped_velocity(path)
        result = make_nav2_parameters(defaults, overrides, layout, stamped)
        ctrl = result['controller_server']['ros__parameters']
        assert ctrl['FollowPath']['plugin'] == 'nav2_mppi_controller::MPPIController'
        assert abs(ctrl['controller_frequency']*ctrl['FollowPath']['model_dt']-1) < 1e-6
        assert ctrl['enable_stamped_cmd_vel'] == stamped
        assert result['waypoint_follower']['ros__parameters']['wait_at_waypoint'][
            'waypoint_pause_duration'] == 5000
        assert result['amcl']['ros__parameters']['base_frame_id'] == 'base_footprint'
        spawn = git_text(args.tb3, tb_revision, 'nav2_minimal_tb3_sim/launch/spawn_tb3.launch.py')
        for argument in ['namespace', 'robot_name', 'robot_sdf', 'x_pose', 'y_pose', 'yaw']:
            assert f"'{argument}'" in spawn, argument
        # Verify every file path depended on by our launch exists in each branch.
        urdf = git_text(args.tb3, tb_revision, 'nav2_minimal_tb3_sim/urdf/turtlebot3_waffle.urdf')
        sdf = git_text(args.tb3, tb_revision, 'nav2_minimal_tb3_sim/urdf/gz_waffle.sdf.xacro')
        assert '<child_frame_id>base_footprint</child_frame_id>' in sdf
        assert ('<frame_id>base_scan</frame_id>' in sdf or
                '<gz_frame_id>base_scan</gz_frame_id>' in sdf)
        changed_sdf = ET.fromstring(adapt_sdf(sdf, layout['robot']))
        changed_urdf = ET.fromstring(adapt_urdf(urdf, layout['robot']))
        sdf_size = changed_sdf.findtext("model/link[@name='base_link']/collision/geometry/box/size")
        urdf_size = changed_urdf.find("link[@name='base_link']/collision/geometry/box").get('size')
        assert sdf_size == urdf_size == '0.5 0.5 0.089'
        wheel = changed_sdf.findtext("model/joint[@name='wheel_left_joint']/pose")
        assert float(wheel.split()[1]) == .235
        git_text(args.nav2, nav_revision, 'nav2_bringup/launch/bringup_launch.py')
        git_text(args.nav2, nav_revision, 'nav2_bringup/launch/rviz_launch.py')
        git_text(args.nav2, nav_revision, 'nav2_bringup/rviz/nav2_default_view.rviz')
        action = git_text(args.nav2, nav_revision, 'nav2_msgs/action/FollowWaypoints.action')
        assert 'missed_waypoints' in action and 'current_waypoint' in action
        reports.append(dict(distro=distro, nav2_commit=nav_revision, tb3_commit=tb_revision,
                            cmd_vel='TwistStamped' if stamped else 'Twist',
                            default_controller=ctrl['FollowPath']['plugin'],
                            resized_robot='0.50 x 0.50 m: SDF/URDF/wheel-track verified',
                            contract_checks='passed', dynamic_simulation='not_run'))
    print(json.dumps(reports, indent=2))


if __name__ == '__main__':
    main()
