#!/usr/bin/env python3
"""Render the actual generated map and waypoint geometry for review."""
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from inventory_inspection_poc.configuration import load_layout
from inventory_inspection_poc.geometry import obstacles


def main():
    layout = load_layout(ROOT/'config/layout.yaml')
    fig, axis = plt.subplots(figsize=(8, 8), layout='constrained')
    axis.set_facecolor('#f5f7fa')
    for box in obstacles(layout):
        wall = box['name'].startswith('wall_')
        # View from Home towards +x, matching the user's Gazebo screenshot:
        # screen-horizontal = -y, screen-vertical = +x.
        u, v = -box['y'], box['x']
        axis.add_patch(Rectangle((u-box['size_y']/2, v-box['size_x']/2),
                                 box['size_y'], box['size_x'],
                                 facecolor='#374151' if wall else '#b77941', zorder=3))
        if not wall:
            axis.text(u, v, box['name'].replace('_', ' '),
                      color='white', ha='center', va='center', weight='bold', zorder=4,
                      rotation=90 if box['size_x'] > box['size_y'] else 0)
    poses = [layout['home'], *layout['inspection_waypoints']]
    for pose in poses:
        u, v = -pose['y'], pose['x']
        axis.scatter(u, v, s=110, color='#0284c7', zorder=5)
        axis.annotate(pose['name'] + (' (back)' if pose['name'] == 'C' else ''),
                      (u, v), xytext=(10, 9),
                      textcoords='offset points', weight='bold', fontsize=12)
        axis.arrow(u, v, -.5*math.sin(pose['yaw']), .5*math.cos(pose['yaw']),
                    width=.035, head_width=.17, color='#0284c7', zorder=5)
    home = layout['home']
    axis.add_patch(Rectangle((-home['y']-.25, home['x']-.25), .5, .5,
                             fill=False, edgecolor='#0284c7', linewidth=2,
                             label='0.50 x 0.50 m robot envelope', zorder=5))
    axis.set(xlim=(-8, 8), ylim=(-8, 8), xlabel='-y (m)', ylabel='x (m)',
              title='15 m x 15 m warehouse | Home > A > B > C > Home\n'
                    'View from Home: A left, B right, C far shelf / stop behind C')
    axis.set_aspect('equal')
    axis.grid(alpha=.15)
    axis.legend(loc='lower center', fontsize=8)
    output = ROOT.parents[1]/'docs'
    output.mkdir(exist_ok=True)
    fig.savefig(output/'warehouse_layout.png', dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
