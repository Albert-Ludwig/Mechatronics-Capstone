"""Meaningful checks that need no ROS, Gazebo, GPU or network."""
import copy
from importlib.util import module_from_spec, spec_from_file_location
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch, Mock
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from inventory_inspection_poc.acceptance import check_action_result, DwellTracker
from inventory_inspection_poc.configuration import (
    detect_stamped_velocity, load_layout, make_nav2_parameters)
from inventory_inspection_poc.geometry import clearance, obstacles
from inventory_inspection_poc.robot_model import adapt_sdf, adapt_urdf, dimensions

spec = spec_from_file_location('asset_generator', ROOT/'tools/generate_assets.py')
generator = module_from_spec(spec)
spec.loader.exec_module(generator)


class WarehouseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.layout = load_layout(ROOT/'config/layout.yaml')
        cls.boxes = obstacles(cls.layout)

    def test_generated_assets_are_current_and_match_every_pixel(self):
        expected = generator.build_map(self.layout)
        self.assertEqual((ROOT/'maps/warehouse.pgm').read_bytes(), expected)
        self.assertEqual((ROOT/'worlds/warehouse.sdf').read_bytes(),
                         generator.build_world(self.layout))
        header, width, maximum, pixels = expected.split(b'\n', 3)
        self.assertEqual((header, width, maximum), (b'P5', b'300 300', b'255'))
        self.assertEqual(len(pixels), 90000)
        self.assertIn(0, pixels)
        self.assertIn(254, pixels)

    def test_world_collision_geometry_matches_map_source(self):
        world = ET.parse(ROOT/'worlds/warehouse.sdf').getroot().find('world')
        for box in self.boxes:
            model = world.find(f"model[@name='{box['name']}']")
            pose = list(map(float, model.findtext('pose').split()))
            dims = list(map(float, model.findtext('link/collision/geometry/box/size').split()))
            self.assertEqual(pose[:2], [box['x'], box['y']])
            self.assertEqual(dims, [box['size_x'], box['size_y'], box['height']])
            self.assertIsNotNone(model.find('link/sensor/contact/collision'))
            self.assertEqual(model.findtext('link/sensor/contact/topic'),
                             '/inspection_contacts/'+box['name'])
        self.assertEqual(len(self.boxes), 7)
        self.assertEqual(world.findall('model[@name="marker_A"]/link/collision'), [])

    def test_shelves_have_required_dimensions(self):
        horizontal = [s for s in self.layout['shelves'] if s['size_x'] == 5 and s['size_y'] == 1]
        vertical = [s for s in self.layout['shelves'] if s['size_x'] == 1 and s['size_y'] == 5]
        self.assertEqual((len(horizontal), len(vertical)), (2, 1))

    def test_waypoints_face_their_shelves_with_one_metre_clearance(self):
        for pose, shelf in zip(self.layout['inspection_waypoints'], self.layout['shelves']):
            self.assertAlmostEqual(clearance(pose['x'], pose['y'], shelf), 1.0)
            direction = math.atan2(shelf['y']-pose['y'], shelf['x']-pose['x'])
            self.assertAlmostEqual(direction, pose['yaw'])

    def test_all_mission_legs_connected_with_055_m_clearance(self):
        # Independent flood search on a 0.10 m grid with conservative 0.55 m
        # obstacle expansion. This checks feasibility, not Nav2 runtime behavior.
        from collections import deque
        size, res = 150, .1
        free = set()
        for row in range(size):
            for col in range(size):
                x, y = -7.5+(col+.5)*res, -7.5+(row+.5)*res
                if min(clearance(x, y, box) for box in self.boxes) > .55:
                    free.add((col, row))
        def cell(pose):
            return int((pose['x']+7.5)/res), int((pose['y']+7.5)/res)
        home = cell(self.layout['home'])
        seen = {home}
        queue = deque([home])
        while queue:
            col, row = queue.popleft()
            for neighbor in ((col+1,row), (col-1,row), (col,row+1), (col,row-1)):
                if neighbor in free and neighbor not in seen:
                    seen.add(neighbor)
                    queue.append(neighbor)
        for pose in self.layout['inspection_waypoints']:
            self.assertIn(cell(pose), seen, pose['name'] + ' is unreachable')


class ParameterTests(unittest.TestCase):
    def test_default_uses_nav2_approach_protection_without_fixed_circles(self):
        overrides = yaml.safe_load((ROOT/'config/nav2_overrides.yaml').read_text())
        layout = load_layout(ROOT/'config/layout.yaml')
        result = make_nav2_parameters({'amcl': {'ros__parameters': {}}}, overrides, layout, True)
        monitor = result['collision_monitor']['ros__parameters']
        self.assertTrue(monitor['enabled'])
        self.assertTrue(monitor['scan']['enabled'])
        self.assertTrue(monitor['FootprintApproach']['enabled'])
        self.assertEqual(monitor['FootprintApproach']['time_before_collision'], 2.0)
        for name in ('StopCircle', 'SlowCircle'):
            self.assertFalse(monitor[name]['enabled'])
            self.assertFalse(monitor[name]['visualize'])

    def test_nav2_only_disables_all_monitor_inputs_without_changing_navigation(self):
        overrides = yaml.safe_load((ROOT/'config/nav2_overrides.yaml').read_text())
        defaults = {'amcl': {'ros__parameters': {}}, 'collision_monitor': {
            'ros__parameters': {'enabled': True, 'observation_sources': ['scan', 'depth'],
                               'depth': {'type': 'pointcloud', 'enabled': True}}}}
        layout = load_layout(ROOT/'config/layout.yaml')
        protected = make_nav2_parameters(defaults, overrides, layout, True)
        unprotected = make_nav2_parameters(defaults, overrides, layout, True, False)
        monitor = unprotected.pop('collision_monitor')['ros__parameters']
        self.assertFalse(monitor['enabled'])
        for name in monitor['polygons'] + monitor['observation_sources']:
            self.assertFalse(monitor[name]['enabled'])
        self.assertTrue(protected.pop('collision_monitor')['ros__parameters']['enabled'])
        self.assertEqual(unprotected, protected)
        self.assertTrue(defaults['collision_monitor']['ros__parameters']['depth']['enabled'])

    def test_robot_keeps_one_metre_limit_when_pedestrian_slows(self):
        layout = load_layout(ROOT/'config/layout.yaml')
        params = yaml.safe_load((ROOT/'config/nav2_overrides.yaml').read_text())
        limit = layout['robot']['max_linear_velocity']
        self.assertEqual(limit, 1.)
        self.assertEqual(layout['pedestrian']['speed'], .6)
        self.assertEqual(params['controller_server']['ros__parameters']['FollowPath']['vx_max'], limit)
        smoother = params['velocity_smoother']['ros__parameters']
        self.assertEqual(smoother['max_velocity'][0], limit)
        self.assertLessEqual(smoother['max_accel'][0], layout['robot']['max_linear_acceleration'])
        self.assertLessEqual(abs(smoother['max_decel'][0]), layout['robot']['max_linear_deceleration'])

    def test_square_footprint_and_global_circle_cover_the_robot(self):
        overrides = yaml.safe_load((ROOT/'config/nav2_overrides.yaml').read_text())
        layout = load_layout(ROOT/'config/layout.yaml')
        result = make_nav2_parameters({'amcl': {'ros__parameters': {}}}, overrides, layout, True)
        local = result['local_costmap']['local_costmap']['ros__parameters']
        global_map = result['global_costmap']['global_costmap']['ros__parameters']
        polygon = yaml.safe_load(local['footprint'])
        self.assertEqual(max(p[0] for p in polygon)-min(p[0] for p in polygon), .5)
        self.assertEqual(max(p[1] for p in polygon)-min(p[1] for p in polygon), .5)
        self.assertNotIn('robot_radius', local)
        corner = math.hypot(.25+local['footprint_padding'], .25+local['footprint_padding'])
        self.assertGreaterEqual(global_map['robot_radius'], corner)
        self.assertLess(global_map['robot_radius']-corner, .01)
        self.assertTrue(result['controller_server']['ros__parameters']['FollowPath'][
            'CostCritic']['consider_footprint'])

    def test_no_untyped_empty_filters_are_emitted(self):
        overrides = yaml.safe_load((ROOT/'config/nav2_overrides.yaml').read_text())
        for inherited_filters in (None, ['keepout_filter', 'speed_filter']):
            costmap = {'robot_radius': .22}
            if inherited_filters is not None:
                costmap['filters'] = inherited_filters
            defaults = {
                'amcl': {'ros__parameters': {}},
                'local_costmap': {'local_costmap': {'ros__parameters': copy.deepcopy(costmap)}},
                'global_costmap': {'global_costmap': {'ros__parameters': copy.deepcopy(costmap)}},
            }
            result = make_nav2_parameters(defaults, overrides,
                                         load_layout(ROOT/'config/layout.yaml'), True)
            for name in ('local_costmap', 'global_costmap'):
                parameters = result[name][name]['ros__parameters']
                self.assertNotEqual(parameters.get('filters'), [])
                self.assertEqual(parameters.get('filters'), inherited_filters)

    def test_merge_preserves_upstream_plugin_and_detects_velocity_type(self):
        defaults = {
            'amcl': {'ros__parameters': {'scan_topic': 'scan'}},
            'controller_server': {'ros__parameters': {
                'FollowPath': {'plugin': 'nav2_mppi_controller::MPPIController',
                               'model_dt': .05, 'future_version_option': True}}},
            'collision_monitor': {'ros__parameters': {'source_timeout': 1.}},
        }
        original = copy.deepcopy(defaults)
        for stamped in (False, True):
            with tempfile.TemporaryDirectory() as tmp:
                bridge = Path(tmp)/'bridge.yaml'
                kind = 'TwistStamped' if stamped else 'Twist'
                bridge.write_text(yaml.safe_dump([dict(topic_name='cmd_vel',
                    ros_type_name='geometry_msgs/msg/'+kind)]))
                self.assertEqual(detect_stamped_velocity(bridge), stamped)
            overrides = yaml.safe_load((ROOT/'config/nav2_overrides.yaml').read_text())
            result = make_nav2_parameters(defaults, overrides,
                                         load_layout(ROOT/'config/layout.yaml'), stamped)
            ctrl = result['controller_server']['ros__parameters']
            self.assertEqual(ctrl['enable_stamped_cmd_vel'], stamped)
            self.assertEqual(ctrl['FollowPath']['plugin'], 'nav2_mppi_controller::MPPIController')
            self.assertTrue(ctrl['FollowPath']['future_version_option'])
            self.assertEqual(ctrl['controller_frequency'], 1/ctrl['FollowPath']['model_dt'])
            wait = result['waypoint_follower']['ros__parameters']['wait_at_waypoint']
            self.assertEqual(wait['waypoint_pause_duration'], 5000)
            self.assertTrue(result['amcl']['ros__parameters']['use_sim_time'])
        self.assertEqual(defaults, original)


class AcceptanceTests(unittest.TestCase):
    def test_succeeded_action_with_missed_waypoints_is_failure(self):
        with self.assertRaises(RuntimeError):
            check_action_result(4, NS(error_code=0, missed_waypoints=[NS(waypoint_index=1)]), True)

    def test_aborted_cancelled_and_error_results_fail(self):
        for status, code in ((6, 0), (5, 0), (4, 603)):
            with self.assertRaises(RuntimeError):
                check_action_result(status, NS(error_code=code, error_msg='test'))

    def test_valid_action_succeeds(self):
        check_action_result(4, NS(error_code=0, missed_waypoints=[]), True)

    def test_stationary_dwell_is_measured_not_assumed(self):
        tracker = DwellTracker([dict(name='A', x=0., y=0., yaw=0.)])
        for i in range(27):
            tracker.observe(i*.2, 0., 0., 0., 0., 0., 0)
        self.assertTrue(tracker.results()[0]['dwell_observed'])

    def test_motion_wrong_heading_and_data_gaps_do_not_count_as_dwell(self):
        for speed, yaw, gap in ((.3, 0., False), (0., 1., False), (0., 0., True)):
            tracker = DwellTracker([dict(name='A', x=0., y=0., yaw=0.)])
            for i in range(30):
                stamp = i*.8 if gap else i*.2
                tracker.observe(stamp, 0., 0., yaw, speed, 0., 0)
            self.assertFalse(tracker.results()[0]['dwell_observed'])


class RobotModelTests(unittest.TestCase):
    def test_running_gear_fits_in_configured_envelope(self):
        robot = load_layout(ROOT/'config/layout.yaml')['robot']
        length, width, height, track, mass = dimensions(robot)
        self.assertEqual((length, width), (.5, .5))
        self.assertLess(track+.018, width)
        self.assertLess(height, .12)
        self.assertGreater(mass, 0)

    def test_impossible_width_is_rejected(self):
        robot = load_layout(ROOT/'config/layout.yaml')['robot']
        robot['width'] = .3
        with self.assertRaises(ValueError):
            dimensions(robot)

    def test_changed_upstream_schema_fails_with_a_useful_error(self):
        robot = load_layout(ROOT/'config/layout.yaml')['robot']
        with self.assertRaisesRegex(ValueError, 'missing'):
            adapt_sdf('<sdf><model/></sdf>', robot)
        with self.assertRaisesRegex(ValueError, 'missing'):
            adapt_urdf('<robot/>', robot)

    def test_c_moved_behind_shelf_and_direct_route_is_blocked(self):
        layout = load_layout(ROOT/'config/layout.yaml')
        c = layout['inspection_waypoints'][2]
        shelf = layout['shelves'][2]
        self.assertEqual((c['x'], c['y']), (5.5, 0.))
        self.assertAlmostEqual(c['yaw'], math.pi)
        self.assertGreater(c['x'], shelf['x']+shelf['size_x']/2)
        # A straight Home-to-C line crosses the shelf. Nav2 must plan a detour.
        self.assertEqual(clearance(4., 0., shelf), 0.)


def load_mission_with_fake_ros():
    """Import production orchestration with stub ROS types; no fake Gazebo claims."""
    modules = {}
    def fake_module(name, **members):
        modules[name] = NS(**members)
    for name in ('action_msgs', 'geometry_msgs', 'lifecycle_msgs', 'nav2_msgs', 'nav_msgs',
                 'ros_gz_interfaces', 'sensor_msgs', 'std_msgs'):
        fake_module(name)
    fake_module('action_msgs.msg', GoalStatus=NS(STATUS_SUCCEEDED=4))
    fake_module('geometry_msgs.msg', PoseStamped=object, PoseWithCovarianceStamped=object,
                Twist=object, TwistStamped=object)
    fake_module('lifecycle_msgs.srv', GetState=object)
    fake_module('nav2_msgs.action', FollowWaypoints=NS(Goal=NS), NavigateToPose=NS(Goal=NS))
    fake_module('nav_msgs.msg', Odometry=object)
    fake_module('rclpy', ok=lambda: True, spin_once=lambda *a, **kw: None)
    fake_module('rclpy.action', ActionClient=object)
    fake_module('rclpy.node', Node=object)
    fake_module('rclpy.qos', DurabilityPolicy=object, QoSProfile=object, qos_profile_sensor_data=None)
    fake_module('ros_gz_interfaces.msg', Contacts=object)
    fake_module('sensor_msgs.msg', LaserScan=object)
    fake_module('std_msgs.msg', String=object)
    fake_module('tf2_ros', Buffer=object, TransformException=Exception, TransformListener=object)
    spec = spec_from_file_location('inventory_inspection_poc._mission_test',
                                  ROOT/'inventory_inspection_poc/mission.py')
    mod = module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(mod)
    return mod


class MissionOrchestrationTests(unittest.TestCase):
    def test_contact_aborts_wait_instead_of_continuing_to_later_goals(self):
        _, node = self.mission()
        node.collisions = [{'topic': '/inspection_contacts/inspection_pedestrian'}]
        node.emergency_stop = Mock()
        with self.assertRaisesRegex(RuntimeError, 'Contact detected'):
            node.wait(lambda: True, 100., 'irrelevant')
        node.emergency_stop.assert_called_once()

    def test_enabled_pedestrian_requires_recent_successful_motion_heartbeat(self):
        mod, node = self.mission()
        node.pedestrian_enabled = True
        node.pedestrian_last_ready = None
        with patch.object(mod.time, 'monotonic', return_value=10.):
            self.assertFalse(node.pedestrian_is_ready())
            node.pedestrian_ready_callback(NS(data=True))
            self.assertTrue(node.pedestrian_is_ready())
        with patch.object(mod.time, 'monotonic', return_value=16.):
            self.assertFalse(node.pedestrian_is_ready())
        node.pedestrian_ready_callback(NS(data=False))
        self.assertIsNone(node.pedestrian_last_ready)
        node.pedestrian_enabled = False
        self.assertTrue(node.pedestrian_is_ready())

    def mission(self):
        mod = load_mission_with_fake_ros()
        node = mod.InspectionMission.__new__(mod.InspectionMission)
        layout = load_layout(ROOT/'config/layout.yaml')
        node.initialize = Mock()
        node.event = Mock()
        node.pose = lambda pose: pose['name']
        node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1000000000))
        node.targets, node.home = layout['inspection_waypoints'], layout['home']
        node.waypoints, node.navigate = 'follower', 'navigator'
        node.collisions = []
        return mod, node

    def test_success_sends_ordered_inspections_then_home(self):
        mod, node = self.mission()
        calls = []
        node.execute = lambda client, goal, label, **kwargs: calls.append((client, goal, kwargs))
        with patch.object(mod.time, 'monotonic', side_effect=[0., 1.]):
            node.run()
        self.assertEqual(calls[0][1].poses, ['A', 'B', 'C'])
        self.assertTrue(calls[0][2]['waypoint_action'])
        self.assertEqual(calls[1][1].pose, 'Home')
        self.assertTrue(node.return_succeeded)
        self.assertEqual(node.phase, 'complete')

    def test_failed_inspections_never_send_home(self):
        _, node = self.mission()
        node.execute = Mock(side_effect=RuntimeError('missed waypoint'))
        with self.assertRaises(RuntimeError):
            node.run()
        self.assertEqual(node.execute.call_count, 1)
        self.assertEqual(node.phase, 'inspections')

    def test_startup_timeout_is_bounded(self):
        mod, node = self.mission()
        with patch.object(mod.time, 'monotonic', return_value=10.):
            with self.assertRaises(TimeoutError):
                node.wait(lambda: False, 5., 'timed out')


if __name__ == '__main__':
    unittest.main(verbosity=2)
