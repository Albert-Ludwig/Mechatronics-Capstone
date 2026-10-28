"""Geometry shared by warehouse generation and acceptance checks."""
import math


def obstacles(layout):
    w = layout['warehouse']
    half, thick = w['size'] / 2.0, w['wall_thickness']
    height = w['wall_height']
    # Warehouse outside dimensions are exactly 15 x 15 m.
    walls = [
        dict(name='wall_north', x=0., y=half-thick/2, size_x=w['size'], size_y=thick),
        dict(name='wall_south', x=0., y=-half+thick/2, size_x=w['size'], size_y=thick),
        dict(name='wall_east', x=half-thick/2, y=0., size_x=thick, size_y=w['size']),
        dict(name='wall_west', x=-half+thick/2, y=0., size_x=thick, size_y=w['size']),
    ]
    return [dict(wall, height=height) for wall in walls] + [
        dict(shelf, height=w['shelf_height']) for shelf in layout['shelves']]


def clearance(x, y, box):
    dx = max(abs(x-box['x']) - box['size_x']/2, 0.)
    dy = max(abs(y-box['y']) - box['size_y']/2, 0.)
    return math.hypot(dx, dy)


def angle_error(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))
