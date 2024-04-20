from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odom_server',
            executable='server',
            name='odom_server',
            # output={
            #     # 'stdout': 'screen',
            #     'stderr': 'screen',      
            # }
        ),
        Node(
            package='odometry_estimate',
            executable='odom',
            name='odometry_estimate',
            output={
                # 'stdout': 'screen',
                'stderr': 'screen',
            }
        )
    ])