"""Drive an independent force-limited pedestrian; no robot pose input."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

from .configuration import load_layout
from .pedestrian import COMMAND_TOPIC, HEADING_TOPIC, JOINT_NAME, STATE_TOPIC, PatrolTarget, validate_path


class PedestrianMotion(Node):
    def __init__(self):
        super().__init__('inspection_pedestrian_motion')
        self.declare_parameter('layout_file', '')
        self.spec = validate_path(load_layout(self.get_parameter('layout_file').value))
        self.patrol = PatrolTarget(self.spec)
        self.command = self.create_publisher(Float64, COMMAND_TOPIC, 10)
        self.heading = self.create_publisher(Float64, HEADING_TOPIC, 10)
        self.ready = self.create_publisher(Bool, '/inspection/pedestrian_ready', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/inspection/pedestrian_pose', 10)
        self.create_subscription(JointState, STATE_TOPIC, self.state_callback, qos_profile_sensor_data)
        self.actual_x = None
        self.state_stamp = None
        self.last_sim = None
        self.waiting = None
        self.create_timer(.01, self.tick)
        self.get_logger().info('Independent pedestrian patrol: avoidance belongs to Nav2')

    def state_callback(self, msg):
        if JOINT_NAME not in msg.name:
            return
        index = msg.name.index(JOINT_NAME)
        if index >= len(msg.position) or not math.isfinite(msg.position[index]):
            return
        self.actual_x = self.spec['x_min']+msg.position[index]
        self.state_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def tick(self):
        now = self.get_clock().now()
        seconds = now.nanoseconds/1e9
        if seconds <= 0 or seconds == self.last_sim:
            return
        dt = 0. if self.last_sim is None else seconds-self.last_sim
        if dt < 0:
            self.patrol = PatrolTarget(self.spec)
            self.actual_x = None
            self.state_stamp = None
        self.last_sim = seconds
        if self.actual_x is None:
            self.ready.publish(Bool(data=False))
            self.command.publish(Float64(data=0.))
            return
        # /clock and joint feedback are separate bridges; allow one clock tick
        # of ordering skew instead of repeatedly braking on fresh feedback.
        fresh = self.state_stamp is not None and -.05 <= seconds-self.state_stamp < .5
        target = self.patrol.step(self.actual_x, dt, fresh=fresh)
        # Joint displacement, not world pose. The physics engine applies <= 8 N.
        self.command.publish(Float64(data=float(target-self.spec['x_min'])))
        self.heading.publish(Float64(data=float(self.patrol.heading)))
        self.ready.publish(Bool(data=fresh))
        if self.waiting != self.patrol.waiting:
            self.waiting = self.patrol.waiting
            self.get_logger().info('Pedestrian feedback stale; holding' if self.waiting
                                   else 'Pedestrian walking independently')
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = now.to_msg()
        pose.pose.position.x, pose.pose.position.y = self.actual_x, float(self.spec['y'])
        pose.pose.orientation.w = 1.
        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PedestrianMotion()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
