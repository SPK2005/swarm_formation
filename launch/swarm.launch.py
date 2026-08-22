from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    leader = LaunchConfiguration('leader')
    rviz = LaunchConfiguration('rviz')
    n_drones = LaunchConfiguration('n_drones')
    log_dir = LaunchConfiguration('log_dir')

    # 'figure8' -> executable 'figure8_leader', etc.
    leader_exec = PythonExpression(["'", leader, "' + '_leader'"])

    common_params = [{
        'n_drones': n_drones,
        'log_dir': log_dir,
    }]

    return LaunchDescription([

        DeclareLaunchArgument(
            'leader', default_value='figure8',
            choices=['figure8', 'circle', 'straight_line'],
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

        Node(package='swarm_formation', executable=leader_exec,
             name='leader', output='screen', parameters=common_params),

        Node(package='swarm_formation', executable='formation_manager',
             name='formation_manager', output='screen', parameters=common_params),

        Node(package='swarm_formation', executable='follower_controller',
             name='follower_controller', output='screen', parameters=common_params),

        Node(package='swarm_formation', executable='swarm_visualizer',
             name='swarm_visualizer', output='screen', parameters=common_params),

        Node(package='swarm_formation', executable='plotter',
             name='plotter', output='screen', parameters=common_params),

        ExecuteProcess(cmd=['rviz2'], output='screen',
                       condition=IfCondition(rviz)),
    ])
