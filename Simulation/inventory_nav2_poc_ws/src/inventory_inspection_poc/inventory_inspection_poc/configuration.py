"""Pure Python configuration helpers, also exercised without ROS installed."""
import copy
import math
from pathlib import Path

import yaml


def load_layout(path):
    data = yaml.safe_load(Path(path).read_text())
    if len(data['shelves']) != 3 or len(data['inspection_waypoints']) != 3:
        raise ValueError('The POC requires exactly three shelves and three inspection poses')
    for pose in [data['home'], *data['inspection_waypoints']]:
        if not all(math.isfinite(float(pose[key])) for key in ('x', 'y', 'yaw')):
            raise ValueError('Pose contains a non-finite number')
    return data


def deep_merge(base, patch):
    result = copy.deepcopy(base)
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def detect_stamped_velocity(bridge_path):
    bridges = yaml.safe_load(Path(bridge_path).read_text())
    for bridge in bridges:
        topic = bridge.get('ros_topic_name', bridge.get('topic_name', ''))
        if topic.strip('/') == 'cmd_vel':
            kind = bridge['ros_type_name']
            if kind not in ('geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped'):
                raise ValueError(f'Unsupported cmd_vel message type: {kind}')
            return kind.endswith('TwistStamped')
    raise ValueError('No cmd_vel mapping in official TurtleBot3 bridge configuration')


def make_nav2_parameters(defaults, overrides, layout, stamped, extra_protection=True):
    params = deep_merge(defaults, overrides)
    robot = layout.get('robot')
    if robot:
        length, width = float(robot['length']), float(robot['width'])
        padding = float(robot['footprint_padding'])
        local = params['local_costmap']['local_costmap']['ros__parameters']
        local.pop('robot_radius', None)
        local['footprint'] = str([[length/2, width/2], [length/2, -width/2],
                                  [-length/2, -width/2], [-length/2, width/2]])
        local['footprint_padding'] = padding
        global_map = params['global_costmap']['global_costmap']['ros__parameters']
        global_map.pop('footprint', None)
        # NavFn checks cells rather than heading-dependent polygon collisions.
        # Round up the padded corner radius to 0.01 m, not down to half-width.
        radius = math.hypot(length/2+padding, width/2+padding)
        global_map['robot_radius'] = math.ceil(radius*100)/100
        global_map['footprint_padding'] = 0.0
    home = layout['home']
    params['amcl']['ros__parameters']['initial_pose'] = {
        'x': float(home['x']), 'y': float(home['y']), 'z': 0.0,
        'yaw': float(home['yaw']),
    }
    for node in ('controller_server', 'behavior_server', 'velocity_smoother',
                 'collision_monitor', 'bt_navigator'):
        if node in params:
            params[node]['ros__parameters']['enable_stamped_cmd_vel'] = stamped

    def simulation_time(tree):
        if not isinstance(tree, dict):
            return
        for key, value in tree.items():
            if key == 'ros__parameters':
                value['use_sim_time'] = True
            else:
                simulation_time(value)
    simulation_time(params)
    if not extra_protection:
        # Keep the official launch's velocity relay, but remove every source of
        # collision-monitor intervention (including invalid-source stopping).
        monitor = params['collision_monitor']['ros__parameters']
        monitor['enabled'] = False
        for name in monitor['polygons']:
            monitor[name]['enabled'] = False
            monitor[name]['visualize'] = False
        for name in monitor.get('observation_sources', ['scan']):
            monitor.setdefault(name, {})['enabled'] = False
    return params
