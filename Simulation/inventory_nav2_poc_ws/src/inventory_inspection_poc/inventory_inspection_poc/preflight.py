"""Read-only package/interface checks. Run before opening the complete simulation."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from .configuration import detect_stamped_velocity
from .configuration import load_layout
from .robot_model import adapt_sdf, adapt_urdf


def main():
    distro = os.environ.get('ROS_DISTRO', '')
    if distro not in ('lyrical', 'jazzy'):
        raise SystemExit('Source ROS 2 Lyrical or Jazzy first. This POC does not target Humble/Fortress.')
    print('ROS_DISTRO=' + distro)
    required = {
        'nav2_bringup': ['launch/bringup_launch.py', 'launch/rviz_launch.py', 'params/nav2_params.yaml'],
        'nav2_minimal_tb3_sim': ['launch/spawn_tb3.launch.py', 'urdf/gz_waffle.sdf.xacro',
                                  'urdf/turtlebot3_waffle.urdf',
                                  'configs/turtlebot3_waffle_bridge.yaml'],
        'ros_gz_sim': ['launch/gz_sim.launch.py'], 'ros_gz_bridge': [],
        'nav2_waypoint_follower': [], 'nav2_amcl': [], 'nav2_mppi_controller': [],
        'rviz2': [], 'robot_state_publisher': [], 'xacro': [],
    }
    for package, files in required.items():
        directory = Path(get_package_share_directory(package))
        for file in files:
            if not (directory/file).is_file():
                raise SystemExit(f'Missing official asset: {directory/file}')
        print('OK ' + package)
    bridge = Path(get_package_share_directory('nav2_minimal_tb3_sim'))/'configs/turtlebot3_waffle_bridge.yaml'
    print('cmd_vel=' + ('TwistStamped' if detect_stamped_velocity(bridge) else 'Twist'))
    share = Path(get_package_share_directory('inventory_inspection_poc'))
    spec = load_layout(share/'config/layout.yaml')['robot']
    tb3 = Path(get_package_share_directory('nav2_minimal_tb3_sim'))
    adapt_sdf((tb3/'urdf/gz_waffle.sdf.xacro').read_text(), spec)
    adapt_urdf((tb3/'urdf/turtlebot3_waffle.urdf').read_text(), spec)
    print(f"Robot body: {spec['length']} x {spec['width']} m; wheel track: {spec['wheel_separation']} m")
    from nav2_msgs.action import FollowWaypoints, NavigateToPose
    from ros_gz_interfaces.msg import Contacts
    assert hasattr(FollowWaypoints.Goal(), 'poses')
    assert hasattr(NavigateToPose.Goal(), 'pose')
    assert hasattr(Contacts(), 'contacts')
    print('Preflight passed. Runtime navigation still needs to be tested.')
