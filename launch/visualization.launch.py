"""Attach read-only visualization to an existing simulator or rosbag."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def visualizer_node(context):
    overrides = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    for name in ('drone_state_source', 'acceleration_mode'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value
    return [Node(package='hih_rviz', executable='hih_visualizer', name='hih_rviz', output='screen',
                 parameters=[LaunchConfiguration('config'), overrides])]


def generate_launch_description():
    share = FindPackageShare('hih_rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([share, 'config', 'visualization.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Empty means use this value from config, including a custom YAML file.
        DeclareLaunchArgument('drone_state_source', default_value=''),
        DeclareLaunchArgument('acceleration_mode', default_value=''),
        OpaqueFunction(function=visualizer_node),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'handinhand.rviz'])],
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
