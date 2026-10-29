"""Official TurtleBot3 + AMCL/Nav2 + an unattended inspection mission."""
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from inventory_inspection_poc.configuration import (
    detect_stamped_velocity, load_layout, make_nav2_parameters)
from inventory_inspection_poc.pedestrian import (
    COMMAND_TOPIC, HEADING_TOPIC, STATE_TOPIC, VISUAL_TOPIC, contact_topics, world_with_pedestrian)
from inventory_inspection_poc.robot_model import adapt_sdf, adapt_urdf


def start(context):
    share = Path(get_package_share_directory('inventory_inspection_poc'))
    nav_share = Path(get_package_share_directory('nav2_bringup'))
    sim_share = Path(get_package_share_directory('nav2_minimal_tb3_sim'))
    bridge_path = sim_share/'configs/turtlebot3_waffle_bridge.yaml'
    layout = load_layout(share/'config/layout.yaml')
    home = layout['home']
    pedestrian_enabled = IfCondition(LaunchConfiguration('pedestrian')).evaluate(context)
    extra_protection = IfCondition(LaunchConfiguration('extra_protection')).evaluate(context)
    stamped = detect_stamped_velocity(bridge_path)
    defaults = yaml.safe_load((nav_share/'params/nav2_params.yaml').read_text())
    overrides = yaml.safe_load((share/'config/nav2_overrides.yaml').read_text())
    params = make_nav2_parameters(defaults, overrides, layout, stamped, extra_protection)

    # This directory lives until shutdown; no race with Gazebo reading the world.
    temporary = tempfile.TemporaryDirectory(prefix='inventory_nav2_')
    world_path = share/'worlds/warehouse.sdf'
    if pedestrian_enabled:
        moving_world = Path(temporary.name)/'warehouse_pedestrian.sdf'
        moving_world.write_bytes(world_with_pedestrian(world_path.read_bytes(), layout))
        world_path = moving_world
    robot_sdf = Path(temporary.name)/'inspection_tb3.sdf.xacro'
    robot_sdf.write_text(adapt_sdf(
        (sim_share/'urdf/gz_waffle.sdf.xacro').read_text(), layout['robot']))
    robot_description = adapt_urdf(
        (sim_share/'urdf/turtlebot3_waffle.urdf').read_text(), layout['robot'])
    param_file = Path(temporary.name)/'nav2_params.yaml'
    param_file.write_text(yaml.safe_dump(params, sort_keys=False))
    contacts_file = Path(temporary.name)/'contacts.yaml'
    contacts_file.write_text(yaml.safe_dump([
        {'topic_name': topic,
         'ros_type_name': 'ros_gz_interfaces/msg/Contacts',
         'gz_type_name': 'gz.msgs.Contacts', 'direction': 'GZ_TO_ROS'}
        for topic in contact_topics(layout, pedestrian_enabled)]))
    cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=lambda context: temporary.cleanup())]))

    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    gz_args = '-r -v 2 ' + ('-s ' if headless else '') + str(world_path)
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items())

    # Keep the official robot, wheel/laser frames, odometry and bridge together.
    robot = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(sim_share/'launch/spawn_tb3.launch.py')),
        launch_arguments={
            'namespace': '', 'robot_name': 'turtlebot3_waffle',
            'robot_sdf': str(robot_sdf),
            'x_pose': str(home['x']), 'y_pose': str(home['y']),
            'z_pose': '0.01', 'roll': '0.0', 'pitch': '0.0',
            'yaw': str(home['yaw']), 'use_sim_time': 'true',
        }.items())
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        output='screen')
    navigation = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(nav_share/'launch/bringup_launch.py')),
        launch_arguments={
            'namespace': '', 'slam': 'False', 'use_localization': 'True',
            'serve_static_map': 'True', 'map': str(share/'maps/warehouse.yaml'),
            'use_sim_time': 'True', 'params_file': str(param_file),
            'autostart': 'True', 'use_composition': 'False',
            'use_respawn': 'False', 'use_keepout_zones': 'False',
            'use_speed_zones': 'False', 'log_level': 'info',
        }.items())
    rviz = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(nav_share/'launch/rviz_launch.py')),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        launch_arguments={'namespace': '', 'use_sim_time': 'True',
                          'rviz_config': str(nav_share/'rviz/nav2_default_view.rviz')}.items())
    contacts = Node(package='ros_gz_bridge', executable='parameter_bridge',
                    name='inspection_contact_bridge', output='screen',
                    parameters=[{'config_file': str(contacts_file), 'use_sim_time': True}])
    mission = Node(
        package='inventory_inspection_poc', executable='inspection_mission',
        name='inspection_mission', output='screen',
        condition=IfCondition(LaunchConfiguration('run_mission')),
        parameters=[{
            'use_sim_time': True, 'layout_file': str(share/'config/layout.yaml'),
            'pedestrian_enabled': pedestrian_enabled,
            'extra_protection': extra_protection,
            'enable_stamped_cmd_vel': stamped,
            'results_dir': LaunchConfiguration('results_dir'),
            'startup_timeout': 240.0, 'mission_timeout': 900.0,
        }])
    pedestrian_nodes = []
    if pedestrian_enabled:
        pedestrian_nodes = [
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='inspection_pedestrian_bridge', output='screen',
                 arguments=[COMMAND_TOPIC+'@std_msgs/msg/Float64]gz.msgs.Double',
                            HEADING_TOPIC+'@std_msgs/msg/Float64]gz.msgs.Double',
                            STATE_TOPIC+'@sensor_msgs/msg/JointState[gz.msgs.Model',
                            VISUAL_TOPIC+'@geometry_msgs/msg/PoseStamped[gz.msgs.Pose']),
            Node(package='inventory_inspection_poc', executable='pedestrian_motion',
                 output='screen', parameters=[{
                     'use_sim_time': True, 'layout_file': str(share/'config/layout.yaml')}]),
        ]
    # The server must inherit resource paths BEFORE it starts (not just the spawner).
    resources = [AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(share/'models')),
                 AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(sim_share/'models')),
                 AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(sim_share.parent))]
    if pedestrian_enabled:
        visual_share = Path(get_package_share_directory('inventory_pedestrian_visual'))
        resources.append(AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',
                                                   str(visual_share.parent.parent/'lib')))
    return [cleanup, *resources, LogInfo(msg=f'Using resized TurtleBot3 (0.50 x 0.50 m); stamped cmd_vel={stamped}'),
            LogInfo(msg=f'Effective Nav2 parameters: {param_file}'),
            LogInfo(msg=f'Pedestrian enabled: {pedestrian_enabled}'),
            LogInfo(msg=f'Extra collision-monitor protection: {extra_protection}'),
            gazebo, robot, state_publisher, contacts, *pedestrian_nodes, navigation, rviz, mission]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Hide Gazebo GUI; sensors still use rendering'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('run_mission', default_value='true'),
        DeclareLaunchArgument('extra_protection', default_value='true',
                              description='Enable official Nav2 footprint approach protection; false disables the monitor'),
        DeclareLaunchArgument('pedestrian', default_value='false',
                              description='Enable bounded pedestrian patrol in the central aisle'),
        DeclareLaunchArgument('results_dir', default_value=str(Path.home()/'inventory_poc_results')),
        OpaqueFunction(function=start),
    ])
