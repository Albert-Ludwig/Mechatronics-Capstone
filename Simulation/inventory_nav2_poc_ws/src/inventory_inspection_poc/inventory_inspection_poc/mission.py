"""Bounded startup -> FollowWaypoints(A,B,C) -> NavigateToPose(Home).

Nav2 WaitAtWaypoint performs each five-second inspection pause. Motion is owned
by Nav2; a detected contact cancels the mission and publishes an emergency zero.
"""
import csv
from datetime import datetime, timezone
from functools import partial
import json
import math
from pathlib import Path
import subprocess
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .acceptance import check_action_result, DwellTracker
from .configuration import load_layout
from .geometry import angle_error
from .pedestrian import contact_topics


class InspectionMission(Node):
    def __init__(self):
        super().__init__('inspection_mission')
        self.declare_parameter('layout_file', '')
        self.declare_parameter('results_dir', str(Path.home()/'inventory_poc_results'))
        self.declare_parameter('startup_timeout', 240.0)
        self.declare_parameter('mission_timeout', 900.0)
        self.declare_parameter('pedestrian_enabled', False)
        self.declare_parameter('extra_protection', True)
        self.extra_protection = self.get_parameter('extra_protection').value
        self.pedestrian_enabled = self.get_parameter('pedestrian_enabled').value
        self.pedestrian_last_ready = None
        self.declare_parameter('enable_stamped_cmd_vel', True)
        self.stamped_velocity = self.get_parameter('enable_stamped_cmd_vel').value
        self.velocity_type = TwistStamped if self.stamped_velocity else Twist
        self.stop_pub = self.create_publisher(self.velocity_type, '/cmd_vel', 10)
        self.layout = load_layout(self.get_parameter('layout_file').value)
        self.home = self.layout['home']
        self.targets = self.layout['inspection_waypoints']
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.result_dir = Path(self.get_parameter('results_dir').value).expanduser()/stamp
        self.result_dir.mkdir(parents=True)
        self.started_wall = time.monotonic()
        self.mission_start_sim = None
        self.phase = 'startup'
        self.events = []
        self.collisions = []
        self.contact_samples = set()
        self.contact_publishers_registered = set()
        self.amcl_pose = None
        self.odom = None
        self.scan_received = None
        self.odom_received = None
        self.target_index = None
        self.dwell = DwellTracker(self.targets)
        self.last_tf_stamp = None
        self.last_map_pose = None
        self.active_goal = None
        self.inspections_succeeded = False
        self.return_succeeded = False
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.waypoints = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.navigate = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.status_pub = self.create_publisher(String, '/inspection/status', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.amcl_callback, QoSProfile(
                                     depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if self.pedestrian_enabled:
            from std_msgs.msg import Bool
            self.create_subscription(Bool, '/inspection/pedestrian_ready',
                                     self.pedestrian_ready_callback, 10)
        for topic in contact_topics(self.layout, self.pedestrian_enabled):
            self.create_subscription(Contacts, topic, partial(self.contact_callback, topic),
                                     qos_profile_sensor_data)
        self.trajectory_file = (self.result_dir/'trajectory.csv').open('w', newline='')
        self.trajectory = csv.writer(self.trajectory_file)
        self.trajectory.writerow(['sim_time', 'x_map', 'y_map', 'yaw', 'linear', 'angular', 'phase'])
        self.create_timer(.2, self.sample_pose)

    def event(self, kind, **data):
        entry = dict(event=kind, sim_time=self.get_clock().now().nanoseconds/1e9,
                     wall_elapsed=time.monotonic()-self.started_wall, **data)
        self.events.append(entry)
        self.get_logger().info(json.dumps(entry, ensure_ascii=False))
        self.status_pub.publish(String(data=json.dumps(entry)))

    def scan_callback(self, msg):
        if msg.ranges and any(math.isfinite(v) and msg.range_min <= v <= msg.range_max
                              for v in msg.ranges):
            self.scan_received = time.monotonic()

    def odom_callback(self, msg):
        self.odom = msg
        self.odom_received = time.monotonic()

    def amcl_callback(self, msg):
        self.amcl_pose = msg

    def contact_callback(self, topic, msg):
        self.contact_samples.add(topic)
        if msg.contacts:
            # Deduplicate repeated contact reports from one ongoing collision.
            if not self.collisions or self.collisions[-1]['topic'] != topic:
                record = dict(topic=topic, sim_time=self.get_clock().now().nanoseconds/1e9)
                self.collisions.append(record)
                self.get_logger().error('COLLISION: ' + topic)
            self.emergency_stop()

    def emergency_stop(self):
        command = self.velocity_type()
        if self.stamped_velocity:
            command.header.stamp = self.get_clock().now().to_msg()
        self.stop_pub.publish(command)

    def pedestrian_ready_callback(self, msg):
        self.pedestrian_last_ready = time.monotonic() if msg.data else None

    def pedestrian_is_ready(self):
        return (not self.pedestrian_enabled or
                (self.pedestrian_last_ready is not None and
                 time.monotonic()-self.pedestrian_last_ready < 5.))

    def sample_pose(self):
        if self.odom is None:
            return
        try:
            transform = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec/1e9
        if self.last_tf_stamp == stamp:
            return
        self.last_tf_stamp = stamp
        p, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        linear, angular = self.odom.twist.twist.linear.x, self.odom.twist.twist.angular.z
        self.last_map_pose = (p.x, p.y, yaw)
        self.trajectory.writerow([stamp, p.x, p.y, yaw, linear, angular, self.phase])
        self.trajectory_file.flush()
        now = self.get_clock().now().nanoseconds/1e9
        odom_age = time.monotonic() - self.odom_received
        if abs(now-stamp) < .6 and odom_age < 2.0:
            self.dwell.observe(stamp, p.x, p.y, yaw, linear, angular,
                               self.target_index if self.phase == 'inspections' else None)

    def wait(self, predicate, deadline, message):
        while rclpy.ok():
            if self.collisions and not getattr(self, 'cancelling', False):
                self.emergency_stop()
                raise RuntimeError('Contact detected; navigation aborted immediately')
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(message)
            rclpy.spin_once(self, timeout_sec=.1)
        raise InterruptedError('ROS shutdown requested')

    def future_result(self, future, deadline, message):
        self.wait(future.done, deadline, message)
        return future.result()

    def wait_active(self, name, deadline):
        service = self.create_client(GetState, name + '/get_state')
        try:
            self.wait(service.service_is_ready, deadline, f'{name}/get_state unavailable')
            while True:
                reply = self.future_result(service.call_async(GetState.Request()), deadline,
                                           f'Timeout reading {name} lifecycle state')
                if reply.current_state.id == 3:  # lifecycle ACTIVE
                    return
                # Keep processing callbacks while waiting for the lifecycle manager.
                until = min(time.monotonic()+.3, deadline)
                while time.monotonic() < until and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=.1)
                if time.monotonic() >= deadline:
                    raise TimeoutError(f'{name} never became ACTIVE')
        finally:
            self.destroy_client(service)

    def pose(self, data):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(data['x'])
        msg.pose.position.y = float(data['y'])
        msg.pose.orientation.z = math.sin(data['yaw']/2)
        msg.pose.orientation.w = math.cos(data['yaw']/2)
        return msg

    def initialize(self):
        deadline = time.monotonic()+self.get_parameter('startup_timeout').value
        self.event('waiting_for_navigation')
        self.wait(lambda: self.get_clock().now().nanoseconds > 0, deadline,
                  'No /clock: check Gazebo is running and not paused')
        self.wait(self.pedestrian_is_ready, deadline,
                  'Pedestrian not moving: check pedestrian_motion and set_pose bridge')
        for name in ('map_server', 'amcl', 'bt_navigator', 'waypoint_follower'):
            self.wait_active(name, deadline)
            self.event('node_active', node=name)
        # The AMCL parameters also set Home; publishing removes dependence on RViz.
        initial = PoseWithCovarianceStamped()
        initial.header = self.pose(self.home).header
        initial.pose.pose = self.pose(self.home).pose
        initial.pose.covariance[0] = .04
        initial.pose.covariance[7] = .04
        initial.pose.covariance[35] = .04
        self.initial_pub.publish(initial)
        self.wait(lambda: self.scan_received is not None and self.odom_received is not None
                  and self.amcl_pose is not None and self.last_map_pose is not None
                  and time.monotonic()-self.scan_received < 5.
                  and time.monotonic()-self.odom_received < 5., deadline,
                  'Localization/sensor startup timeout: check /scan /odom and map->base_footprint TF')
        x, y, yaw = self.last_map_pose
        if math.hypot(x-self.home['x'], y-self.home['y']) > .5:
            raise RuntimeError('Initial map pose is not near Home; restart the complete simulation')
        if angle_error(yaw, self.home['yaw']) > .4:
            raise RuntimeError('Initial heading does not match Home; check localization')
        # Contact sensors publish only ON collision, so requiring an empty sample
        # from every sensor would make collision-free acceptance impossible.
        # Check Gazebo publisher discovery separately from actual contact events.
        try:
            topic_list = subprocess.run(['gz', 'topic', '-l'], capture_output=True,
                                        text=True, timeout=8., check=True).stdout.splitlines()
            expected = set(contact_topics(self.layout, self.pedestrian_enabled))
            self.contact_publishers_registered = expected.intersection(
                topic.strip() for topic in topic_list)
        except (OSError, subprocess.SubprocessError) as exc:
            self.get_logger().warning(f'Cannot verify Gazebo contact publisher discovery: {exc}')
        self.event('localization_ready', x=x, y=y, yaw=yaw)

    def feedback(self, message):
        index = int(message.feedback.current_waypoint)
        if index != self.target_index:
            self.target_index = index
            if index < len(self.targets):
                self.event('navigating_to', waypoint=self.targets[index]['name'], index=index)

    def execute(self, client, goal, label, waypoint_action=False):
        deadline = time.monotonic()+self.get_parameter('mission_timeout').value
        self.wait(client.server_is_ready, deadline, f'{label}: action server unavailable')
        kwargs = {'feedback_callback': self.feedback} if waypoint_action else {}
        handle = self.future_result(client.send_goal_async(goal, **kwargs), deadline,
                                    label + ': goal acceptance timed out')
        if not handle.accepted:
            raise RuntimeError(label + ': goal rejected by Nav2')
        self.active_goal = handle
        result = self.future_result(handle.get_result_async(), deadline,
                                    label + ': navigation timed out')
        self.active_goal = None
        check_action_result(result.status, result.result, waypoint_action)
        self.event('action_succeeded', action=label)

    def run(self):
        self.initialize()
        self.phase = 'inspections'
        self.mission_start_sim = self.get_clock().now().nanoseconds/1e9
        self.event('mission_started', sequence=['Home', 'A', 'B', 'C', 'Home'])
        goal = FollowWaypoints.Goal()
        goal.poses = [self.pose(pose) for pose in self.targets]
        # number_of_loops=0 executes the list once; it does not mean skip it.
        if hasattr(goal, 'number_of_loops'):
            goal.number_of_loops = 0
        self.execute(self.waypoints, goal, 'A -> B -> C', waypoint_action=True)
        self.inspections_succeeded = True
        self.phase = 'return_home'
        self.target_index = None
        self.event('returning_home')
        home_goal = NavigateToPose.Goal()
        home_goal.pose = self.pose(self.home)
        self.execute(self.navigate, home_goal, 'Return Home')
        # Process fresh telemetry once after the action result before verification.
        until = time.monotonic() + .5
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=.1)
        self.return_succeeded = True
        if self.collisions:
            raise RuntimeError('Navigation completed but warehouse contact sensors reported a collision')
        self.phase = 'complete'
        self.event('mission_complete')

    def save_report(self, error=None):
        now = self.get_clock().now().nanoseconds/1e9
        pose = self.last_map_pose
        home_distance = None if pose is None else math.hypot(
            pose[0]-self.home['x'], pose[1]-self.home['y'])
        home_yaw = None if pose is None else angle_error(pose[2], self.home['yaw'])
        dwell = self.dwell.results()
        expected = len(contact_topics(self.layout, self.pedestrian_enabled))
        report = dict(
            extra_protection=self.extra_protection,
            pedestrian_enabled=self.pedestrian_enabled,
            pedestrian_motion_ready=self.pedestrian_is_ready(),
            navigation_success=error is None and self.inspections_succeeded and self.return_succeeded,
            error=error, phase=self.phase,
            sequence=['Home', 'A', 'B', 'C', 'Home'],
            mission_sim_seconds=None if self.mission_start_sim is None else now-self.mission_start_sim,
            total_wall_seconds=time.monotonic()-self.started_wall,
            configured_dwell_seconds=5.0, dwell_measurements=dwell,
            inspections_succeeded=self.inspections_succeeded,
            return_action_succeeded=self.return_succeeded,
            final_home_distance_m=home_distance, final_home_yaw_error_rad=home_yaw,
            collision_events=self.collisions,
            contact_topics_with_samples=sorted(self.contact_samples),
            contact_publishers_registered=sorted(self.contact_publishers_registered),
            contact_monitor_coverage=len(self.contact_publishers_registered)/expected,
            acceptance_passed=(error is None and self.pedestrian_is_ready()
                               and self.inspections_succeeded and self.return_succeeded
                               and all(d['dwell_observed'] for d in dwell)
                               and home_distance is not None and home_distance <= .25
                               and home_yaw is not None and home_yaw <= .30
                               and len(self.contact_publishers_registered) == expected and not self.collisions),
            note='Full dynamic acceptance additionally requires three fresh complete simulation runs. '
                 'Contact sensors are event-only. Coverage uses Gazebo publisher discovery, '
                 'not empty contact messages. Visually verify the three demonstration runs too.',
            events=self.events,
        )
        path = self.result_dir/'mission_report.json'
        path.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
        self.trajectory_file.close()
        self.get_logger().info('Report: ' + str(path))
        self.get_logger().info('navigation_success=' + str(report['navigation_success']) +
                               ' acceptance_passed=' + str(report['acceptance_passed']))

    def cancel_active(self):
        if self.active_goal is not None and rclpy.ok():
            self.cancelling = True
            future = self.active_goal.cancel_goal_async()
            try:
                self.future_result(future, time.monotonic()+3., 'Cancel timeout')
            except (TimeoutError, InterruptedError):
                pass
            finally:
                self.cancelling = False


def main(args=None):
    rclpy.init(args=args)
    node = InspectionMission()
    error = None
    try:
        node.run()
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or 'Interrupted by operator'
        node.get_logger().error(error)
        node.cancel_active()
    finally:
        node.save_report(error)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if error is not None:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
