from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='swarm_formation',
            executable='figure8_leader',
            output='screen'
        ),

        Node(
            package='swarm_formation',
            executable='formation_manager',
            output='screen'
        ),

        Node(
            package='swarm_formation',
            executable='follower_controller',
            output='screen'
        ),
        
        Node(
            package='swarm_formation',
            executable='swarm_visualizer',
            output='screen'
        ),

        Node(
            package='swarm_formation',
            executable='plotter',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['rviz2'],
            output='screen'
        )
    ])
