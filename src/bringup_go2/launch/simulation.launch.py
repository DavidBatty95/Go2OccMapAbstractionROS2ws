from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = dict(world='partitioned', duration='120.0', seed='1', speed='1.0',
                    dose_budget='2.0', radiation_weight='1.0', gui='false', output='/workspace/results/latest')
    declarations = [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
    value = lambda name, kind: ParameterValue(LaunchConfiguration(name), value_type=kind)
    output = {'output': LaunchConfiguration('output')}
    sim = Node(package='go2_occmap', executable='simulator', output='screen', parameters=[output, {
        'world': LaunchConfiguration('world'), 'duration': value('duration', float), 'speed': value('speed', float),
        'seed': value('seed', int), 'gui': value('gui', bool)}])
    mapper = Node(package='go2_occmap', executable='mapper', output='screen',
                  parameters=[output, {'use_sim_time': True}])
    monitor = Node(package='go2_occmap', executable='monitor', output='screen', parameters=[output])
    explorer = Node(package='go2_occmap', executable='explorer', output='screen', parameters=[output, {
        'use_sim_time': True, 'duration': value('duration', float), 'dose_budget': value('dose_budget', float),
        'radiation_weight': value('radiation_weight', float)}])
    rviz_config = str(Path(get_package_share_directory('bringup_go2'))/'config/simulation.rviz')
    rviz = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('gui')))
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason='A critical simulation node exited'))]))
                for node in (sim, mapper, explorer, monitor)]
    return LaunchDescription([*declarations, *handlers, monitor, mapper, explorer, sim, rviz])
