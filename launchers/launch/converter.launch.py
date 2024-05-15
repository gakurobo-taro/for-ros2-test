from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy'
        ),
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
        
        Node(
            package='remote_emergency_stop',
            executable='res',
            name='remote_emergency_stop',
            output={
                # 'stdout': 'screen',
                'stderr': 'screen',
                },
        )
    ])