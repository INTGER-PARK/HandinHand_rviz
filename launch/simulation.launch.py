"""One existing physics world with GT export, original role-local stacks and RViz.

Equivalent to HandinHand_sim's cooperation_launch.py at 9b1e82e, with its plant
entry point replaced by hih_plant_telemetry. Never run both simulation launches.
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def boolean(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    plant_config = PathJoinSubstitution([FindPackageShare('hih_plant'), 'config', 'cooperation_high_fidelity.yaml'])
    own_share = FindPackageShare('hih_rviz')
    nodes = [Node(package='hih_rviz', executable='hih_plant_telemetry', name='cooperation_plant',
                  output='screen', parameters=[plant_config, {
                      'enable_viewer': boolean('viewer'), 'enable_cop_touch': boolean('cop_touch'),
                      'enable_cop_solver': boolean('cop_solver'), 'enable_high_fidelity': True}])]
    for role in ('leader', 'follower'):
        nodes.extend([
            Node(package='hih_controller', executable=role + '_wrench_controller', output='screen',
                 parameters=[{'hold_initial_position': True}]),
            Node(package='hih_controller', executable='hih_allocator_controller', namespace=role, output='screen'),
            Node(package='hih_controller', executable=role + '_second_order_momentum_observer', output='screen'),
        ])
    nodes.append(Node(package='hih_controller', executable='follower_admittance_controller', output='screen',
                      condition=IfCondition(LaunchConfiguration('embedded_admittance')),
                      parameters=[{'enabled': boolean('admittance_enabled'), 'auto_approach': True}]))
    nodes.append(Node(package='hih_logging', executable='hih_record_csv', output='screen',
                      condition=IfCondition(LaunchConfiguration('record_data')),
                      arguments=['--output-dir', LaunchConfiguration('log_output_dir')],
                      sigterm_timeout='120', sigkill_timeout='10'))
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([own_share, 'launch', 'visualization.launch.py'])),
        launch_arguments={'rviz': LaunchConfiguration('rviz'), 'config': LaunchConfiguration('config'),
                          'drone_state_source': LaunchConfiguration('drone_state_source')}.items()))
    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('cop_touch', default_value='false'),
        DeclareLaunchArgument('cop_solver', default_value='true'),
        DeclareLaunchArgument('embedded_admittance', default_value='false'),
        DeclareLaunchArgument('admittance_enabled', default_value='false'),
        DeclareLaunchArgument('record_data', default_value='false'),
        DeclareLaunchArgument('log_output_dir', default_value=str(Path.cwd() / 'hih_recordings')),
        DeclareLaunchArgument('drone_state_source', default_value=''),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([own_share, 'config', 'visualization.yaml'])),
        *nodes,
    ])
