"""Bounded pedestrian path and self-contained Gazebo mannequin geometry."""
import math
import copy
import xml.etree.ElementTree as ET

from .geometry import obstacles

NAME = 'inspection_pedestrian'
ACTOR_NAME = 'inspection_pedestrian_visual'
CONTACT_TOPIC = '/inspection_contacts/' + NAME
COMMAND_TOPIC = '/inspection/pedestrian/position_target'
STATE_TOPIC = '/inspection/pedestrian/joint_states'
JOINT_NAME = 'patrol_joint'
HEADING_TOPIC = '/inspection/pedestrian/heading_target'
VISUAL_TOPIC = '/inspection/pedestrian/visual_pose'


def validate_path(layout):
    spec = layout['pedestrian']
    keys = ('x_min', 'x_max', 'y', 'speed', 'radius', 'clearance',
            'ramp_seconds', 'turn_seconds', 'turn_rate', 'animation_seconds_per_metre')
    if not all(math.isfinite(float(spec[key])) for key in keys):
        raise ValueError('Pedestrian settings must be finite')
    low, high, y = (float(spec[key]) for key in ('x_min', 'x_max', 'y'))
    radius, margin = float(spec['radius']), float(spec['clearance'])
    if high <= low or float(spec['speed']) <= 0 or radius < .28 or margin < 0:
        raise ValueError('Invalid pedestrian limits, speed, radius or clearance')
    if min(spec['ramp_seconds'], spec['turn_rate'], spec['animation_seconds_per_metre']) <= 0:
        raise ValueError('Pedestrian smoothing settings must be positive')
    if spec['turn_seconds'] < math.pi/spec['turn_rate'] + .7:
        raise ValueError('Turn dwell must allow a complete smooth 180 degree turn')
    half = layout['warehouse']['size']/2 - layout['warehouse']['wall_thickness']
    if max(abs(low), abs(high), abs(y)) + radius + margin >= half:
        raise ValueError('Pedestrian path leaves warehouse bounds')
    # Exact minimum distance between the entire horizontal segment and each box.
    # Endpoint-only checks would miss a shelf crossing the middle of the path.
    for box in obstacles(layout):
        left, right = box['x']-box['size_x']/2, box['x']+box['size_x']/2
        dx = max(left-high, low-right, 0.)
        dy = max(abs(y-box['y'])-box['size_y']/2, 0.)
        if math.hypot(dx, dy) <= radius + margin:
            raise ValueError('Pedestrian path intersects safety margin of ' + box['name'])
    return spec


class PatrolTarget:
    """Cosine speed ramps, a full stop, then a timed turn at each endpoint."""
    def __init__(self, spec):
        self.spec = spec
        self.target = float(spec['x_min'])
        self.direction = 1.
        self.heading = 0.
        self.waiting = False
        self.phase = 'leg'
        self.elapsed = 0.
        self.start = None
        self.previous_actual = None

    def begin_leg(self, actual_x):
        cfg = self.spec
        self.start = actual_x
        self.end = cfg['x_max']-.05 if self.direction > 0 else cfg['x_min']+.05
        self.distance = abs(self.end-self.start)
        self.ramp = float(cfg['ramp_seconds'])
        self.peak = min(cfg['speed'], self.distance/self.ramp)
        self.cruise = max(0., self.distance-self.peak*self.ramp)/max(self.peak, 1e-9)
        self.elapsed = 0.
        self.phase = 'leg'

    def step(self, actual_x, dt, fresh=True):
        cfg = self.spec
        dt = min(.05, max(0., dt))
        velocity = (0. if self.previous_actual is None or dt <= 0. else
                    (actual_x-self.previous_actual)/dt)
        self.previous_actual = actual_x
        was_waiting = self.waiting
        self.waiting = not fresh
        if self.waiting:
            if not was_waiting:
                self.target = actual_x
            return self.target
        if self.start is None or was_waiting:
            self.begin_leg(actual_x)
        if self.phase == 'turn':
            self.elapsed += dt
            if self.elapsed >= cfg['turn_seconds']:
                self.begin_leg(actual_x)
        elif self.phase == 'settle':
            if abs(actual_x-self.end) < .035 and abs(velocity) < .06:
                self.direction *= -1.
                self.heading = 0. if self.direction > 0 else math.pi
                self.phase = 'turn'
                self.elapsed = 0.
        else:
            self.elapsed += dt
            t, ramp, peak = self.elapsed, self.ramp, self.peak
            if t < ramp:
                distance = .5*peak*(t-ramp/math.pi*math.sin(math.pi*t/ramp))
            elif t < ramp+self.cruise:
                distance = .5*peak*ramp+peak*(t-ramp)
            else:
                u = min(ramp, t-ramp-self.cruise)
                distance = (.5*peak*ramp+peak*self.cruise +
                            .5*peak*(u+ramp/math.pi*math.sin(math.pi*u/ramp)))
            self.target = self.start+self.direction*min(self.distance, distance)
            if t >= 2*ramp+self.cruise:
                self.target = self.end
                self.phase = 'settle'
        # Bound following error without replacing the smooth velocity profile.
        lead = max(.18, cfg['speed']*.70)
        self.target = min(actual_x+lead, max(actual_x-lead, self.target))
        self.target = min(cfg['x_max'], max(cfg['x_min'], self.target))
        return self.target


def contact_topics(layout, enabled=False):
    topics = ['/inspection_contacts/' + box['name'] for box in obstacles(layout)]
    return topics + ([CONTACT_TOPIC] if enabled else [])


def world_with_pedestrian(world_bytes, layout):
    spec = validate_path(layout)
    sdf = ET.fromstring(world_bytes)
    world = sdf.find('world')
    if world is None or world.find(f"model[@name='{NAME}']") is not None:
        raise ValueError('Expected one world without an existing pedestrian')
    def sub(parent, tag, text=None, **attrs):
        item = ET.SubElement(parent, tag, attrs)
        if text is not None:
            item.text = str(text)
        return item
    model = sub(world, 'model', name=NAME)
    # Dynamic body on a physical slider, driven by bounded force. Never teleport
    # a static collision into the robot: penetration correction can launch it.
    sub(model, 'static', 'false')
    sub(model, 'pose', f"{spec['x_min']} {spec['y']} 0 0 0 0")
    link = sub(model, 'link', name='body')
    sub(link, 'gravity', 'false')
    inertial = sub(link, 'inertial')
    sub(inertial, 'pose', '0 0 .9 0 0 0')
    sub(inertial, 'mass', '10')
    inertia = sub(inertial, 'inertia')
    for key, value in [('ixx', 2.8), ('iyy', 2.8), ('izz', .55),
                       ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
        sub(inertia, key, value)
    # Primitive human silhouette; no online model downloads or mesh dependencies.
    parts = [
        ('torso', 'box', '0 0 1.10 0 0 0', '0.30 0.38 0.60', '0.9 0.35 0.06 1'),
        ('head', 'sphere', '0 0 1.57 0 0 0', '.15', '0.85 0.65 0.45 1'),
        ('left_leg', 'cylinder', '0 .12 .43 0 0 0', '.075 .76', '0.08 0.12 0.24 1'),
        ('right_leg', 'cylinder', '0 -.12 .43 0 0 0', '.075 .76', '0.08 0.12 0.24 1'),
        ('left_arm', 'cylinder', '0 .23 1.08 0 0 0', '.045 .56', '0.9 0.35 0.06 1'),
        ('right_arm', 'cylinder', '0 -.23 1.08 0 0 0', '.045 .56', '0.9 0.35 0.06 1'),
        ('nose', 'sphere', '.15 0 1.57 0 0 0', '.035', '0.85 0.65 0.45 1'),
    ]
    collision_names = []
    for name, kind, pose, dimensions, color in parts:
        # Build geometry once for the collision proxy. Rendering is supplied by
        # the matching HuNav skin below instead of primitive mannequin parts.
        visual = ET.Element('visual', name=name)
        sub(visual, 'pose', pose)
        geometry = sub(sub(visual, 'geometry'), kind)
        if kind == 'box':
            sub(geometry, 'size', dimensions)
        elif kind == 'sphere':
            sub(geometry, 'radius', dimensions)
        else:
            radius, length = dimensions.split()
            sub(geometry, 'radius', radius)
            sub(geometry, 'length', length)
        material = sub(visual, 'material')
        sub(material, 'ambient', color)
        sub(material, 'diffuse', color)
        collision_name = name+'_collision'
        collision = sub(link, 'collision', name=collision_name)
        sub(collision, 'pose', pose)
        collision.append(copy.deepcopy(visual.find('geometry')))
        collision_names.append(collision_name)
    sensor = sub(link, 'sensor', name='contact', type='contact')
    sub(sensor, 'always_on', 'true')
    sub(sensor, 'update_rate', '20')
    contact = sub(sensor, 'contact')
    for name in collision_names:
        sub(contact, 'collision', name)
    sub(contact, 'topic', CONTACT_TOPIC)
    joint = sub(model, 'joint', name=JOINT_NAME, type='prismatic')
    sub(joint, 'parent', 'world')
    sub(joint, 'child', 'body')
    axis = sub(joint, 'axis')
    sub(axis, 'xyz', '1 0 0')
    limit = sub(axis, 'limit')
    for key, value in [('lower', 0.), ('upper', spec['x_max']-spec['x_min']),
                       ('effort', 8.), ('velocity', spec['speed'])]:
        sub(limit, key, value)
    sub(sub(axis, 'dynamics'), 'damping', '4')
    controller = sub(model, 'plugin', filename='gz-sim-joint-position-controller-system',
                     name='gz::sim::systems::JointPositionController')
    for key, value in [('joint_name', JOINT_NAME), ('topic', COMMAND_TOPIC),
                       ('p_gain', 80), ('i_gain', 0), ('d_gain', 30),
                       ('cmd_max', 8), ('cmd_min', -8), ('initial_position', 0),
                       ('use_velocity_commands', 'false')]:
        sub(controller, key, value)
    state = sub(model, 'plugin', filename='gz-sim-joint-state-publisher-system',
                name='gz::sim::systems::JointStatePublisher')
    sub(state, 'joint_name', JOINT_NAME)
    sub(state, 'topic', STATE_TOPIC)
    actor = sub(world, 'actor', name=ACTOR_NAME)
    # This skin's feet sit at about z=-0.82 in the native Gazebo actor frame.
    sub(actor, 'pose', f"{spec['x_min']} {spec['y']} 0.85 0 0 0")
    skin = sub(actor, 'skin')
    sub(skin, 'filename', 'model://hunav_worker/worker_man.dae')
    sub(skin, 'scale', '1.0')
    animation = sub(actor, 'animation', name='walk')
    sub(animation, 'filename', 'model://hunav_worker/walk.dae')
    sub(animation, 'scale', '1.0')
    sub(animation, 'interpolate_x', 'true')
    follower = sub(actor, 'plugin', filename='libinventory_pedestrian_visual.so',
                   name='inventory::SmoothActor')
    for key, value in [('target', NAME), ('heading_topic', HEADING_TOPIC),
                       ('yaw_rate', spec['turn_rate']),
                       ('animation_seconds_per_metre', spec['animation_seconds_per_metre'])]:
        sub(follower, key, value)
    ET.indent(sdf, space='  ')
    return ET.tostring(sdf, encoding='utf-8', xml_declaration=True) + b'\n'
