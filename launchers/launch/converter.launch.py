from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='converter',
            executable='vel2steer',
            name='vel2steer',
            output={
                # 'stdout': 'screen',
                'stderr': 'screen',
                },
            emulate_tty=True,
        ),
    ])