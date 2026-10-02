from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    leader = LaunchConfiguration('leader').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)
    n_drones = int(LaunchConfiguration('n_drones').perform(context))
    log_dir = LaunchConfiguration('log_dir').perform(context)
    enable_delta = LaunchConfiguration('enable_delta').perform(context) == 'true'
    log_tag = LaunchConfiguration('log_tag').perform(context)
    leader_speed = LaunchConfiguration('leader_speed').perform(context)
    leader_amplitude = LaunchConfiguration('leader_amplitude').perform(context)
    leader_wavelength = LaunchConfiguration('leader_wavelength').perform(context)

    common_params = {
        'n_drones': n_drones,
        'log_dir': log_dir,
    }

    # 'figure8' -> executable 'figure8_leader', etc.
    leader_params = dict(common_params)
    if leader_speed:
        leader_params['speed'] = float(leader_speed)
    if leader_amplitude:
        leader_params['amplitude'] = float(leader_amplitude)
    if leader_wavelength:
        leader_params['wavelength'] = float(leader_wavelength)

    follower_params = dict(common_params)
    follower_params['enable_delta'] = enable_delta

    # formation_logger / plotter use output_dir + num_followers, not
    # log_dir/n_drones -- match the parameter names they actually declare.
    logger_params = {
        'output_dir': log_dir,
        'num_followers': n_drones,
        'tag': log_tag,
    }
    plotter_params = {
        'output_dir': log_dir,
        'num_followers': n_drones,
    }

    actions = [
        Node(package='swarm_formation', executable=f'{leader}_leader',
             name='leader', output='screen', parameters=[leader_params]),

        Node(package='swarm_formation', executable='formation_manager',
             name='formation_manager', output='screen', parameters=[common_params]),

        Node(package='swarm_formation', executable='follower_controller',
             name='follower_controller', output='screen', parameters=[follower_params]),

        Node(package='swarm_formation', executable='swarm_visualizer',
             name='swarm_visualizer', output='screen', parameters=[common_params]),

        Node(package='swarm_formation', executable='formation_logger',
             name='formation_logger', output='screen', parameters=[logger_params]),

        Node(package='swarm_formation', executable='plotter',
             name='plotter', output='screen', parameters=[plotter_params]),
    ]

    if rviz == 'true':
        actions.append(ExecuteProcess(cmd=['rviz2'], output='screen'))

    return actions


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'leader', default_value='figure8',
            choices=['figure8', 'circle', 'straight_line', 'sinusoid'],
            description='Virtual leader reference path generator.'),

        DeclareLaunchArgument(
            'rviz', default_value='true',
            choices=['true', 'false'],
            description='Launch rviz2 alongside the stack.'),

        DeclareLaunchArgument(
            'n_drones', default_value='4',
            description='Number of follower UAVs (1-4). Must match the '
                        'instance count started by start_swarm.sh.'),

        DeclareLaunchArgument(
            'log_dir', default_value='',
            description='Directory for the formation CSV. Empty = node default.'),

        DeclareLaunchArgument(
            'enable_delta', default_value='true',
            choices=['true', 'false'],
            description="Follower's carousel heading correction (Delta)."),

        DeclareLaunchArgument(
            'log_tag', default_value='',
            description='Suffix appended to the logger CSV filename.'),

        DeclareLaunchArgument(
            'leader_speed', default_value='',
            description='Leader ground speed. Empty = node default.'),

        DeclareLaunchArgument(
            'leader_amplitude', default_value='',
            description='Sinusoid amplitude A. Empty = node default.'),

        DeclareLaunchArgument(
            'leader_wavelength', default_value='',
            description='Sinusoid wavelength. Empty = node default.'),

        OpaqueFunction(function=launch_setup),
    ])
