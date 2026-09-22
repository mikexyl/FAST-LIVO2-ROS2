from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    root = Path(get_package_share_directory('fast_livo'))
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ('Alpha', 'Bob', 'Carol'):
        raise ValueError('robot must be Alpha, Bob, or Carol')
    seq = LaunchConfiguration('seq_name').perform(context) or f'S3E_Square_1_{robot.lower()}'
    camera = str(root / 'config/s3e' / f'{robot.lower()}_camera.yaml')
    config = LaunchConfiguration('mapping_config').perform(context) or str(root / 'config/s3e' / f'{robot.lower()}.yaml')
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    extra = {'use_sim_time': True, 'evo.seq_name': seq,
             'research.robot_id': robot,
             'camera_parameter_node': f'/{namespace}/parameter_blackboard' if namespace else 'parameter_blackboard',
             'output_directory': LaunchConfiguration('output_directory').perform(context),
             'research.output_directory': LaunchConfiguration('export_directory').perform(context),
             'research.python': LaunchConfiguration('export_python').perform(context),
             'research.writer_script': LaunchConfiguration('export_writer').perform(context)}
    if namespace:
        extra.update({'frames.world': f'{robot}/odom', 'frames.body': f'{robot}/imu'})
    mapper = Node(package='fast_livo', executable='fastlivo_mapping', name='laserMapping', namespace=namespace,
                  prefix='gdb -batch -ex run -ex "thread apply all bt" -ex "signal SIGINT" -ex "thread apply all bt" --args' if os.environ.get('FAST_LIVO_GDB') == '1' else None,
                  parameters=[config, extra], output='screen',
                  sigterm_timeout='120' if extra['research.output_directory'] else '5',
                  sigkill_timeout='30' if extra['research.output_directory'] else '10')
    return [
        RegisterEventHandler(OnProcessExit(target_action=mapper,
            on_exit=[EmitEvent(event=Shutdown(reason='FAST-LIVO2 exited'))])),
        Node(package='demo_nodes_cpp', executable='parameter_blackboard',
             name='parameter_blackboard', namespace=namespace, parameters=[camera, {'use_sim_time': True}]),
        Node(package='fast_livo', executable='s3e_adapter.py', namespace=namespace,
             parameters=[{'robot': robot, 'use_sim_time': True,
                          'visualization': LaunchConfiguration('use_rerun'),
                          'camera_config': camera, 'mapping_config': config}], output='screen'),
        mapper,
        Node(package='rviz2', executable='rviz2',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', str(root / 'rviz_cfg/fast_livo2.rviz')],
             parameters=[{'use_sim_time': True}], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='Alpha', choices=['Alpha', 'Bob', 'Carol']),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_rerun', default_value='false'),
        DeclareLaunchArgument('seq_name', default_value=''),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('mapping_config', default_value=''),
        DeclareLaunchArgument('output_directory', default_value=''),
        DeclareLaunchArgument('export_directory', default_value=''),
        DeclareLaunchArgument('export_python', default_value='/usr/bin/python3'),
        DeclareLaunchArgument('export_writer', default_value=''),
        OpaqueFunction(function=setup),
    ])
