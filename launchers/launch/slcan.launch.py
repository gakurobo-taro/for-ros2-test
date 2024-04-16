from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slcan_cpp',
            executable='slcan',
            name='slcan',
            output={
                # 'stdout': 'screen',
                'stderr': 'screen',
                },
            emulate_tty=True,
            parameters=[
                {'slcan_topic_name': 'slcan'},
                {'serial_name': '/dev/ttyACM0',},
                {'baudrate', '115200'}
            ]
        ),
    ])