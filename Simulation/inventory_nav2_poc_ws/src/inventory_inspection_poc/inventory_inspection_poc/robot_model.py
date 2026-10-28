"""Resize the installed official TB3 at launch without editing /opt/ros.

Use a box for the enlarged chassis so the physical collision and visible body
are exactly the configured size. Wheel diameter, sensor frames and plugins
stay based on the official model; wheel spacing and joint positions agree.
"""
import math
import xml.etree.ElementTree as ET

ET.register_namespace('xacro', 'http://www.ros.org/wiki/xacro')


def dimensions(spec):
    length, width = float(spec['length']), float(spec['width'])
    height, track = float(spec['height']), float(spec['wheel_separation'])
    mass = float(spec['mass'])
    if not all(math.isfinite(v) and v > 0 for v in (length, width, height, track, mass)):
        raise ValueError('Robot dimensions, wheel separation and mass must be positive')
    # Official TB3 tires are 0.018 m wide, at x=0, and have radius 0.033 m.
    if track + .018 > width or length < .40 or height > .12:
        raise ValueError('Configured chassis does not enclose the TB3 running gear or obscures LiDAR')
    return length, width, height, track, mass


def required(parent, xpath):
    value = parent.find(xpath)
    if value is None:
        raise ValueError('Installed TurtleBot3 model changed; missing ' + xpath)
    return value


def set_text(parent, name, value):
    element = parent.find(name)
    if element is None:
        element = ET.SubElement(parent, name)
    element.text = str(value)
    return element


def adapt_sdf(text, spec):
    length, width, height, track, mass = dimensions(spec)
    root = ET.fromstring(text)
    model = required(root, 'model')
    body = required(model, "link[@name='base_link']")
    collision = required(body, "collision[@name='base_collision']")
    set_text(collision, 'pose', '0 0 0.048 0 0 0')
    required(collision, 'geometry/box/size').text = f'{length} {width} {height}'
    visual = required(body, "visual[@name='base_visual']")
    set_text(visual, 'pose', '0 0 0.048 0 0 0')
    visual.remove(required(visual, 'geometry'))
    ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size').text = (
        f'{length} {width} {height}')
    material = required(visual, 'material')
    set_text(material, 'diffuse', '0.10 0.45 0.85 1')
    set_text(material, 'ambient', '0.10 0.45 0.85 1')
    inertial = required(body, 'inertial')
    # Keep the original rearward COM, inside the wheel/caster support triangle.
    set_text(inertial, 'mass', mass)
    inertia = required(inertial, 'inertia')
    for key, value in (
        ('ixx', mass*(width*width+height*height)/12),
        ('iyy', mass*(length*length+height*height)/12),
        ('izz', mass*(length*length+width*width)/12)):
        set_text(inertia, key, value)
    for side, sign in [('left', 1), ('right', -1)]:
        wheel = required(model, f"link[@name='wheel_{side}_link']")
        for path in ('collision/pose', 'visual/pose', 'inertial/pose'):
            pose = required(wheel, path)
            fields = pose.text.split()
            fields[1] = str(sign*track/2)
            pose.text = ' '.join(fields)
        joint = required(model, f"joint[@name='wheel_{side}_joint']/pose")
        fields = joint.text.split()
        fields[1] = str(sign*track/2)
        joint.text = ' '.join(fields)
    drive = required(model, "plugin[@name='gz::sim::systems::DiffDrive']")
    required(drive, 'wheel_separation').text = str(track)
    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode')


def adapt_urdf(text, spec):
    length, width, height, track, mass = dimensions(spec)
    root = ET.fromstring(text)
    body = required(root, "link[@name='base_link']")
    visual = required(body, 'visual')
    required(visual, 'origin').set('xyz', '0 0 0.048')
    visual.remove(required(visual, 'geometry'))
    ET.SubElement(ET.SubElement(visual, 'geometry'), 'box', size=f'{length} {width} {height}')
    required(visual, 'material').set('name', 'blue')
    collision = required(body, 'collision')
    required(collision, 'origin').set('xyz', '0 0 0.048')
    required(collision, 'geometry/box').set('size', f'{length} {width} {height}')
    inertial = required(body, 'inertial')
    required(inertial, 'origin').set('xyz', '-0.064 0 0.048')
    required(inertial, 'mass').set('value', str(mass))
    inertia = required(inertial, 'inertia')
    for key, value in (
        ('ixx', mass*(width*width+height*height)/12),
        ('iyy', mass*(length*length+height*height)/12),
        ('izz', mass*(length*length+width*width)/12),
        ('ixy', 0), ('ixz', 0), ('iyz', 0)):
        inertia.set(key, str(value))
    for side, sign in [('left', 1), ('right', -1)]:
        origin = required(root, f"joint[@name='wheel_{side}_joint']/origin")
        fields = origin.get('xyz').split()
        fields[1] = str(sign*track/2)
        origin.set('xyz', ' '.join(fields))
    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode')
