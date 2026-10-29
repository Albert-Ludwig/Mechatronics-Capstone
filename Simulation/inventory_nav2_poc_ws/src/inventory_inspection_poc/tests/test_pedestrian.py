"""Path safety, endpoint reversal and optional-world regression checks."""
import copy
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from inventory_inspection_poc.configuration import load_layout
from inventory_inspection_poc.pedestrian import (
    NAME, ACTOR_NAME, CONTACT_TOPIC, contact_topics, PatrolTarget, validate_path, world_with_pedestrian)


class PedestrianTests(unittest.TestCase):
    def setUp(self):
        self.layout = load_layout(ROOT/'config/layout.yaml')

    def test_stale_feedback_holds_target_and_resumes_without_catching_up(self):
        spec = validate_path(self.layout)
        patrol = PatrolTarget(spec)
        actual = -1.
        patrol.target = actual
        self.assertEqual(patrol.step(actual, .05, fresh=False), actual)
        # Physical braking drift must not move the hold target forward.
        self.assertEqual(patrol.step(actual+.08, .05, fresh=False), actual)
        self.assertTrue(patrol.waiting)
        target = patrol.step(actual, 500.)
        self.assertLessEqual(target-actual, spec['speed']*.1+.00001)
        self.assertFalse(patrol.waiting)

    def test_patrol_has_no_robot_input_or_tf_subscription(self):
        import ast
        import inspect
        self.assertEqual(list(inspect.signature(PatrolTarget.step).parameters),
                         ['self', 'actual_x', 'dt', 'fresh'])
        tree = ast.parse((ROOT/'inventory_inspection_poc/pedestrian_motion.py').read_text())
        imports = [node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)]
        self.assertNotIn('tf2_ros', imports)
        self.assertNotIn('nav_msgs.msg', imports)

    def test_target_bounds_and_reversal(self):
        spec = validate_path(self.layout)
        patrol = PatrolTarget(spec)
        actual = spec['x_min']
        positions = []
        for _ in range(1000):
            actual = patrol.step(actual, .05)
            positions.append(actual)
        self.assertLessEqual(max(positions), spec['x_max'])
        self.assertGreaterEqual(min(positions), spec['x_min'])
        self.assertGreater(max(positions), spec['x_max']-.1)
        self.assertTrue(any(b<a for a,b in zip(positions,positions[1:])))

    def test_velocity_ramps_are_continuous_and_turn_happens_at_rest(self):
        spec = validate_path(self.layout)
        patrol = PatrolTarget(spec)
        actual = spec['x_min']
        velocities = []
        stopped_turn_seconds = 0.
        previous_heading = patrol.heading
        heading_changes = 0
        for _ in range(1500):
            target = patrol.step(actual, .02)
            velocity = (target-actual)/.02
            velocities.append(velocity)
            if patrol.phase == 'turn':
                self.assertAlmostEqual(velocity, 0., places=8)
                stopped_turn_seconds += .02
            if patrol.heading != previous_heading:
                self.assertLess(abs(velocity), .01)
                heading_changes += 1
            previous_heading = patrol.heading
            actual = target
        self.assertGreater(heading_changes, 0)
        self.assertGreaterEqual(stopped_turn_seconds, spec['turn_seconds']-.05)
        self.assertLessEqual(max(abs(v) for v in velocities), .600001)
        acceleration = [(b-a)/.02 for a,b in zip(velocities,velocities[1:])]
        self.assertLess(max(abs(a) for a in acceleration), .5)

    def test_rejects_routes_crossing_shelves_and_warehouse_bounds(self):
        for patch in ({'x_max': 5.5}, {'y': 3.}, {'x_min': -8.},
                      {'x_max': -4.}, {'speed': 0.}, {'y': float('nan')},
                      {'radius': .1}):
            with self.subTest(patch=patch):
                layout = copy.deepcopy(self.layout)
                layout['pedestrian'].update(patch)
                with self.assertRaises(ValueError):
                    validate_path(layout)
        # Free endpoints alone are insufficient: a shelf can cross the segment.
        self.layout['shelves'][0].update(x=0., y=0., size_x=1., size_y=1.)
        with self.assertRaisesRegex(ValueError, 'shelf_A'):
            validate_path(self.layout)

    def test_optional_world_has_solid_human_and_contact_monitor(self):
        source = (ROOT/'worlds/warehouse.sdf').read_bytes()
        self.assertIsNone(ET.fromstring(source).find(f"world/model[@name='{NAME}']"))
        world = ET.fromstring(world_with_pedestrian(source, self.layout))
        model = world.find(f"world/model[@name='{NAME}']")
        self.assertEqual(model.findtext('static'), 'false')
        self.assertEqual(model.findtext('joint/axis/limit/effort'), '8.0')
        self.assertEqual(model.findtext('joint/axis/limit/upper'), '6.8')
        controller = model.find("plugin[@name='gz::sim::systems::JointPositionController']")
        self.assertEqual(controller.findtext('use_velocity_commands'), 'false')
        self.assertEqual(controller.findtext('cmd_max'), '8')
        # Skin is a separate animated visual; collision proxies stay physical
        # and no full-height invisible cylinder is reintroduced around the legs.
        self.assertEqual(len(model.findall('link/collision')), 7)
        self.assertEqual(model.findtext("link/collision[@name='left_leg_collision']/geometry/cylinder/radius"), '.075')
        self.assertEqual(model.findtext('link/sensor/contact/topic'), CONTACT_TOPIC)
        self.assertEqual(len(model.findall('link/visual')), 0)
        actor = world.find(f"world/actor[@name='{ACTOR_NAME}']")
        self.assertEqual(actor.findtext('skin/filename'), 'model://hunav_worker/worker_man.dae')
        self.assertEqual(actor.findtext('animation/filename'), 'model://hunav_worker/walk.dae')
        self.assertEqual(actor.findtext('plugin/target'), NAME)
        self.assertEqual(actor.find('plugin').get('name'), 'inventory::SmoothActor')
        self.assertEqual(len(contact_topics(self.layout)), 7)
        self.assertEqual(len(contact_topics(self.layout, True)), 8)
        # Warehouse obstacles and localization map remain unchanged.
        original = ET.fromstring(source)
        for name in [s['name'] for s in self.layout['shelves']]:
            path = f"world/model[@name='{name}']"
            self.assertEqual(ET.tostring(original.find(path)), ET.tostring(world.find(path)))
        self.assertEqual(source, (ROOT/'worlds/warehouse.sdf').read_bytes())

    def test_duplicate_pedestrian_is_rejected(self):
        first = world_with_pedestrian((ROOT/'worlds/warehouse.sdf').read_bytes(), self.layout)
        with self.assertRaises(ValueError):
            world_with_pedestrian(first, self.layout)

    def test_hunav_meshes_and_all_texture_assets_are_packaged(self):
        import hashlib
        import json
        root = ROOT/'models/hunav_worker'
        manifest = json.loads((root/'source-sha256.json').read_text())
        for name, checksum in manifest.items():
            self.assertEqual(hashlib.sha256((root/name).read_bytes()).hexdigest(), checksum, name)
        self.assertTrue((root/'LICENSE').is_file())
        xml = ET.parse(root/'worker_man.dae')
        for node in xml.findall('.//{*}library_images/{*}image/{*}init_from'):
            self.assertTrue((root/node.text).is_file(), node.text)


if __name__ == '__main__':
    unittest.main()
