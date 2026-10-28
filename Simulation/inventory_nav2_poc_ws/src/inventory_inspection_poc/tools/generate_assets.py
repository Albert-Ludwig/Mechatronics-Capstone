#!/usr/bin/env python3
"""Generate matching SDF/PGM/YAML from config/layout.yaml. No ROS required."""
import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from inventory_inspection_poc.configuration import load_layout
from inventory_inspection_poc.geometry import obstacles, clearance


def sub(parent, tag, text=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if text is not None:
        child.text = str(text)
    return child


def box_model(world, box, color):
    model = sub(world, 'model', name=box['name'])
    sub(model, 'static', 'true')
    sub(model, 'pose', f"{box['x']} {box['y']} {box['height']/2} 0 0 0")
    link = sub(model, 'link', name='body')
    for kind in ('collision', 'visual'):
        part = sub(link, kind, name=kind)
        geometry = sub(part, 'geometry')
        sub(sub(geometry, 'box'), 'size', f"{box['size_x']} {box['size_y']} {box['height']}")
        if kind == 'visual':
            material = sub(part, 'material')
            sub(material, 'ambient', color)
            sub(material, 'diffuse', color)
    sensor = sub(link, 'sensor', name='contact', type='contact')
    sub(sensor, 'always_on', 'true')
    sub(sensor, 'update_rate', '10')
    contact = sub(sensor, 'contact')
    sub(contact, 'collision', 'collision')
    sub(contact, 'topic', '/inspection_contacts/' + box['name'])


def build_world(layout):
    sdf = ET.Element('sdf', version='1.6')
    world = sub(sdf, 'world', name='default')
    physics = sub(world, 'physics', name='physics', type='ignored')
    sub(physics, 'max_step_size', '0.002')
    sub(physics, 'real_time_factor', '1.0')
    for kind, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                       ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors'),
                       ('imu', 'Imu'), ('contact', 'Contact')]:
        plugin = sub(world, 'plugin', filename=f'gz-sim-{kind}-system',
                     name='gz::sim::systems::' + name)
        if kind == 'sensors':
            sub(plugin, 'render_engine', 'ogre2')
    scene = sub(world, 'scene')
    sub(scene, 'ambient', '0.6 0.6 0.6 1')
    sub(scene, 'background', '0.8 0.85 0.9 1')
    sub(scene, 'shadows', 'false')
    light = sub(world, 'light', name='sun', type='directional')
    sub(light, 'pose', '0 0 10 0 0 0')
    sub(light, 'diffuse', '0.8 0.8 0.8 1')
    sub(light, 'direction', '-0.5 0.1 -0.9')
    ground = sub(world, 'model', name='ground_plane')
    sub(ground, 'static', 'true')
    link = sub(ground, 'link', name='ground')
    for kind in ('collision', 'visual'):
        part = sub(link, kind, name=kind)
        plane = sub(sub(part, 'geometry'), 'plane')
        sub(plane, 'normal', '0 0 1')
        sub(plane, 'size', '15 15')
        if kind == 'visual':
            material = sub(part, 'material')
            sub(material, 'ambient', '0.7 0.7 0.7 1')
            sub(material, 'diffuse', '0.7 0.7 0.7 1')
    for box in obstacles(layout):
        color = '0.35 0.4 0.45 1' if box['name'].startswith('wall') else '0.65 0.38 0.16 1'
        box_model(world, box, color)
    # Flat visual-only floor markers: they never appear in LiDAR or the occupancy map.
    for pose in [layout['home'], *layout['inspection_waypoints']]:
        model = sub(world, 'model', name='marker_' + pose['name'])
        sub(model, 'static', 'true')
        sub(model, 'pose', f"{pose['x']} {pose['y']} 0.003 0 0 {pose['yaw']}")
        visual = sub(sub(model, 'link', name='link'), 'visual', name='visual')
        sub(sub(sub(visual, 'geometry'), 'box'), 'size', '0.35 0.35 0.004')
        material = sub(visual, 'material')
        sub(material, 'ambient', '0.1 0.8 0.2 1')
        sub(material, 'diffuse', '0.1 0.8 0.2 1')
    ET.indent(sdf, space='  ')
    return ET.tostring(sdf, encoding='utf-8', xml_declaration=True) + b'\n'


def build_map(layout):
    cfg = layout['warehouse']
    res, size = cfg['resolution'], cfg['size']
    width = round(size / res)
    if abs(width * res - size) > 1e-8:
        raise ValueError('Size must be an integer number of cells')
    boxes = obstacles(layout)
    pixels = bytearray()
    for row in range(width):
        y = size/2 - (row + .5)*res  # PGM top row is north, not south.
        for col in range(width):
            x = -size/2 + (col + .5)*res
            pixels.append(0 if any(clearance(x, y, b) < 1e-9 for b in boxes) else 254)
    return f'P5\n{width} {width}\n255\n'.encode() + pixels


def generate(root=ROOT):
    layout = load_layout(root/'config/layout.yaml')
    (root/'worlds').mkdir(exist_ok=True)
    (root/'maps').mkdir(exist_ok=True)
    (root/'worlds/warehouse.sdf').write_bytes(build_world(layout))
    (root/'maps/warehouse.pgm').write_bytes(build_map(layout))
    half = layout['warehouse']['size']/2
    (root/'maps/warehouse.yaml').write_text(
        'image: warehouse.pgm\nmode: trinary\n'
        f"resolution: {layout['warehouse']['resolution']}\n"
        f'origin: [{-half}, {-half}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT)
    generate(parser.parse_args().output)
    print('Generated warehouse.sdf and matching 300 x 300 occupancy map')
