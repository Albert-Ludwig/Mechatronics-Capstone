#!/usr/bin/env python3
"""Isolated Lyrical/Gazebo runtime checks; requires sourced workspace and a display.

Modes: mission verifies complete acceptance; contact deliberately drives the robot
slowly into the pedestrian to check contact reporting and physical stability.
Logs and results stay in a temporary directory. Only owned processes are stopped.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['mission', 'contact'], default='mission')
    parser.add_argument('--timeout', type=float, default=480.)
    parser.add_argument('--extra-protection', choices=['on', 'off'], default='on',
                        help='On verifies footprint approach protection; off verifies pass-through')
    parser.add_argument('--attach', action='store_true',
                        help='Observe an existing default-domain mission; leave it running')
    parser.add_argument('--results-dir', type=Path,
                        help='Existing mission result directory when attaching')
    args = parser.parse_args()
    if args.attach and args.mode != 'mission':
        parser.error('The deliberate contact test must run in an isolated scene')
    if args.extra_protection == 'off' and args.mode != 'mission':
        parser.error('The Nav2-only pass-through experiment requires --mode mission')
    if os.environ.get('ROS_DISTRO') != 'lyrical':
        parser.error('This runtime probe currently supports sourced ROS 2 Lyrical only')
    workspace = Path(__file__).resolve().parents[3]
    logs = Path(tempfile.mkdtemp(prefix='nav2-'+args.mode+'-'))
    if not args.attach:
        # Stay below the Linux ephemeral-port overlap of high DDS domain IDs.
        os.environ.update(ROS_DOMAIN_ID=str(50+os.getpid()%45), GZ_PARTITION=logs.name,
                          ROS_HOME=str(logs/'ros_home'), GZ_HOMEDIR=str(logs/'gz_home'))
    os.environ['ROS_LOG_DIR'] = str(logs/'ros')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped, PoseStamped
    from rosgraph_msgs.msg import Clock
    from ros_gz_interfaces.msg import Contacts
    from std_msgs.msg import Bool
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import JointState
    from nav2_msgs.msg import CollisionMonitorState
    from rcl_interfaces.srv import GetParameters
    processes, handles = [], []
    def start(name, command):
        handle = (logs/name).open('w')
        handles.append(handle)
        process = subprocess.Popen(command, cwd=workspace, stdout=handle,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process
    node = None
    print('Runtime evidence: '+str(logs), flush=True)
    state = dict(clock=None, contacts=[], ready=0, nav2_commands=0,
                 nav2_turn_commands=0, monitor_actions={}, dynamic_costmap_observations=0,
                 nav2_max_forward=0., robot_max_linear_speed=0., pedestrian_max_joint_speed=0.)
    state.update(visual_samples=0, visual_max_yaw_rate=0., visual_turn_samples=0,
                 visual_min_yaw=math.inf, visual_max_yaw=-math.inf, previous_visual=None)
    last_human = None
    samples = []
    max_z, max_tilt, min_distance = 0., 0., math.inf
    report = None
    monitor_flags = None
    parameter_future = None
    parameter_retry = 0.
    velocity_inputs, velocity_outputs = {}, {}
    try:
        launch = None if args.attach else start('launch.log', ['ros2', 'launch', 'inventory_inspection_poc',
                       'inspection.launch.py', 'pedestrian:=true', 'headless:=true',
                       'use_rviz:=false', 'run_mission:='+str(args.mode=='mission').lower(),
                       'extra_protection:='+str(args.extra_protection=='on').lower(),
                       'results_dir:='+str(logs/'results')])
        start('poses.jsonl', ['gz', 'topic', '-e', '-t', '/world/default/pose/info', '--json-output'])
        rclpy.init()
        node = rclpy.create_node('pedestrian_runtime_verification')
        parameter_client = node.create_client(GetParameters, '/collision_monitor/get_parameters')
        flag_names = ['enabled', 'StopCircle.enabled', 'SlowCircle.enabled',
                      'FootprintApproach.enabled', 'scan.enabled']
        pub = node.create_publisher(TwistStamped, '/cmd_vel', 10) if args.mode=='contact' else None
        def command(speed):
            if pub is not None and state['clock'] is not None:
                msg = TwistStamped()
                msg.header.stamp = state['clock']
                msg.twist.linear.x = speed
                pub.publish(msg)
        def contact(msg):
            if msg.contacts:
                state['contacts'].append(time.monotonic())
                if args.mode=='contact':
                    command(0.)
        node.create_subscription(Clock, '/clock', lambda msg: state.update(clock=msg.clock),
                                 qos_profile_sensor_data)
        node.create_subscription(Contacts, '/inspection_contacts/inspection_pedestrian',
                                 contact, qos_profile_sensor_data)
        node.create_subscription(Bool, '/inspection/pedestrian_ready',
                                 lambda msg: state.update(ready=state['ready']+int(msg.data)), 10)
        def nav2_command(msg):
            state['nav2_commands'] += 1
            state['nav2_turn_commands'] += int(abs(msg.twist.angular.z)>.1)
            state['nav2_max_forward'] = max(state['nav2_max_forward'],msg.twist.linear.x)
        def odometry(msg):
            state['robot_max_linear_speed'] = max(state['robot_max_linear_speed'],
                                                  abs(msg.twist.twist.linear.x))
        def pedestrian_state(msg):
            if 'patrol_joint' in msg.name:
                index=msg.name.index('patrol_joint')
                if index<len(msg.velocity):
                    state['pedestrian_max_joint_speed']=max(state['pedestrian_max_joint_speed'],
                                                            abs(msg.velocity[index]))
        def visual_pose(msg):
            t=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            yaw=2.*math.atan2(msg.pose.orientation.z,msg.pose.orientation.w)
            state['visual_samples'] += 1
            state['visual_min_yaw'] = min(state['visual_min_yaw'],yaw)
            state['visual_max_yaw'] = max(state['visual_max_yaw'],yaw)
            previous=state['previous_visual']
            if previous is not None and .005 < t-previous[0] < .2:
                rate=abs(math.remainder(yaw-previous[1],2.*math.pi)/(t-previous[0]))
                state['visual_max_yaw_rate']=max(state['visual_max_yaw_rate'],rate)
                state['visual_turn_samples'] += int(rate>.2)
            state['previous_visual']=(t,yaw)
        def monitor_action(msg):
            key = str(msg.action_type)+':'+msg.polygon_name
            state['monitor_actions'][key] = state['monitor_actions'].get(key,0)+1
        def costmap(msg):
            if last_human is None or last_human[0] >= 2.:
                return
            # The central aisle is free in the static map. Check the pedestrian
            # leg area for newly marked near-lethal cells in Nav2's own map.
            res = msg.info.resolution
            if res<=0: return
            x,y = last_human
            for dx in (-.1,0.,.1):
                for dy in (-.2,0.,.2):
                    col=int((x+dx-msg.info.origin.position.x)/res)
                    row=int((y+dy-msg.info.origin.position.y)/res)
                    if 0<=col<msg.info.width and 0<=row<msg.info.height:
                        if msg.data[row*msg.info.width+col]>=95:
                            state['dynamic_costmap_observations'] += 1
                            return
        node.create_subscription(TwistStamped, '/cmd_vel_nav', nav2_command, 10)
        def capture_velocity(target, msg):
            t=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            target[t]=(msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z)
        node.create_subscription(TwistStamped, '/cmd_vel_smoothed',
                                 lambda msg: capture_velocity(velocity_inputs,msg), 100)
        node.create_subscription(TwistStamped, '/cmd_vel',
                                 lambda msg: capture_velocity(velocity_outputs,msg), 100)
        node.create_subscription(Odometry, '/odom', odometry, qos_profile_sensor_data)
        node.create_subscription(JointState, '/inspection/pedestrian/joint_states',
                                 pedestrian_state, qos_profile_sensor_data)
        node.create_subscription(PoseStamped, '/inspection/pedestrian/visual_pose',visual_pose,10)
        node.create_subscription(CollisionMonitorState, '/collision_monitor_state', monitor_action, 10)
        node.create_subscription(OccupancyGrid, '/global_costmap/costmap', costmap, 10)
        offset, buffer = 0, ''
        decoder = json.JSONDecoder()
        began = last_progress = time.monotonic()
        began_wall = time.time()
        while time.monotonic()-began < args.timeout:
            rclpy.spin_once(node, timeout_sec=.05)
            if parameter_future is not None and parameter_future.done():
                reply = parameter_future.result()
                if len(reply.values)==len(flag_names) and all(v.type==1 for v in reply.values):
                    monitor_flags = {name:v.bool_value for name,v in zip(flag_names,reply.values)}
                parameter_future = None
            if (monitor_flags is None and parameter_future is None and state['ready']>10
                    and time.monotonic()>=parameter_retry and parameter_client.service_is_ready()):
                parameter_future=parameter_client.call_async(GetParameters.Request(names=flag_names))
                parameter_retry=time.monotonic()+1.
            if launch is not None and launch.poll() is not None:
                raise RuntimeError('Launch exited early')
            if args.mode=='contact':
                command(.12 if not state['contacts'] and state['ready']>10 else 0.)
            with (logs/'poses.jsonl').open() as stream:
                stream.seek(offset)
                buffer += stream.read()
                offset = stream.tell()
            while buffer.strip():
                buffer = buffer.lstrip()
                try:
                    msg, consumed = decoder.raw_decode(buffer)
                except json.JSONDecodeError:
                    break
                buffer = buffer[consumed:]
                poses = {p.get('name'):p for p in msg.get('pose', [])}
                human, robot = poses.get('inspection_pedestrian'), poses.get('turtlebot3_waffle')
                if human:
                    p = human.get('position', {})
                    samples.append((p.get('x', 0.), p.get('y', 0.)))
                    last_human = samples[-1]
                    if not (-4.005 <= samples[-1][0] <= 2.805 and abs(samples[-1][1])<.005):
                        raise AssertionError('Physical pedestrian left its bounds')
                if robot:
                    p, q = robot.get('position', {}), robot.get('orientation', {})
                    x,y,z,w = (q.get(k,0.) for k in ('x','y','z','w'))
                    tilt = math.acos(max(-1., min(1., 1.-2.*(x*x+y*y))))
                    max_z = max(max_z, p.get('z',0.))
                    max_tilt = max(max_tilt, tilt)
                    if human and time.monotonic()-began>3.:
                        hp = human.get('position', {})
                        min_distance=min(min_distance, math.hypot(p.get('x',0.)-hp.get('x',0.),
                                                                p.get('y',0.)-hp.get('y',0.)))
                    if max_z>.15 or max_tilt>.35:
                        raise AssertionError('Robot lifted or tipped beyond stability limit')
            if args.mode=='contact' and state['contacts'] and time.monotonic()-state['contacts'][0]>2.:
                break
            results_dir = (args.results_dir or Path.home()/'inventory_poc_results'
                           if args.attach else logs/'results')
            reports = [p for p in results_dir.glob('*/mission_report.json')
                       if p.stat().st_mtime >= began_wall]
            if reports:
                report = json.loads(reports[0].read_text())
                break
            if time.monotonic()-last_progress>15.:
                last_progress = time.monotonic()
                print(json.dumps(dict(elapsed=round(last_progress-began,1), max_z=max_z,
                                      max_tilt=max_tilt, contacts=len(state['contacts']),
                                      last_pedestrian=samples[-1] if samples else None)), flush=True)
        # Actual-contact emergency zeros belong to failure handling, not to the
        # pre-contact navigation experiment. Exclude their timestamps explicitly.
        collisions = report.get('collision_events', []) if report else []
        cutoff = (min(c['sim_time'] for c in collisions)-.1)*1e9 if collisions else math.inf
        paired = [t for t in velocity_inputs.keys() & velocity_outputs.keys() if 0<t<cutoff]
        velocity_error = max((abs(a-b) for t in paired
                              for a,b in zip(velocity_inputs[t],velocity_outputs[t])),default=None)
        passthrough = dict(paired_samples=len(paired), max_abs_velocity_error=velocity_error,
                           moving_pairs=sum(any(abs(v)>.01 for v in velocity_inputs[t]) for t in paired),
                           cutoff_before_contact_seconds=.1 if collisions else None)
        evidence = dict(mode=args.mode, attached=args.attach,
                        extra_protection=args.extra_protection=='on',
                        collision_monitor_flags=monitor_flags, velocity_passthrough=passthrough,
                        pedestrian_samples=len(samples), robot_max_z=max_z,
                        robot_max_tilt_radians=max_tilt, min_center_distance=min_distance,
                        contact_samples=len(state['contacts']), ready_samples=state['ready'],
                        nav2_command_samples=state['nav2_commands'],
                        nav2_max_forward=state['nav2_max_forward'],
                        robot_max_linear_speed=state['robot_max_linear_speed'],
                        pedestrian_max_joint_speed=state['pedestrian_max_joint_speed'],
                        visual_pose_samples=state['visual_samples'],
                        visual_max_yaw_rate=state['visual_max_yaw_rate'],
                        visual_yaw_range=[state['visual_min_yaw'],state['visual_max_yaw']],
                        visual_turn_samples=state['visual_turn_samples'],
                        nav2_turn_command_samples=state['nav2_turn_commands'],
                        nav2_collision_monitor_actions=state['monitor_actions'],
                        dynamic_costmap_observations=state['dynamic_costmap_observations'],
                        pedestrian_x_range=[min(p[0] for p in samples),max(p[0] for p in samples)] if samples else [],
                        mission=report)
        (logs/'verification.json').write_text(json.dumps(evidence,indent=2)+'\n')
        print(json.dumps(evidence,indent=2), flush=True)
        assert samples and state['ready']>10, 'No physical pedestrian state'
        assert monitor_flags is not None, 'Could not verify live collision-monitor parameters'
        assert monitor_flags['enabled'] == (args.extra_protection=='on'), 'Wrong protection mode'
        if args.extra_protection=='on':
            assert monitor_flags == dict(zip(flag_names, [True, False, False, True, True])), \
                'Expected official footprint approach protection with fixed circles disabled'
            assert not any(key.endswith((':StopCircle', ':SlowCircle'))
                           for key in state['monitor_actions']), 'Unexpected fixed-circle action'
        if args.extra_protection=='off':
            assert not any(monitor_flags.values()), 'Collision-monitor intervention still enabled'
            assert passthrough['moving_pairs']>10, 'No moving velocity pairs to verify pass-through'
            assert velocity_error is not None and velocity_error<1e-9, 'Velocity relay altered commands'
            assert not state['monitor_actions'], 'Unexpected collision-monitor action'
        if args.mode=='contact':
            assert state['contacts'], 'No intentional physical contact was observed'
        else:
            assert report and report['acceptance_passed'], 'Full mission did not pass acceptance'
            assert not state['contacts'], 'Unexpected pedestrian collision'
            assert state['nav2_commands']>10, 'No commands observed from Nav2 controller'
            assert state['dynamic_costmap_observations']>0, 'No pedestrian observed in Nav2 costmap'
    finally:
        if node:
            if args.mode=='contact': command(0.)
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None: os.killpg(process.pid, signal.SIGINT)
        for process in reversed(processes):
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()
        for handle in handles: handle.close()


if __name__=='__main__':
    main()
