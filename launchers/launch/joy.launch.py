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
        ),
        Node(
            package='converter',
            executable='ctrler2vel',
            name='ctrler2vel',
            output={
                # 'stdout': 'screen',
                'stderr': 'screen',
                },
        ),
        
        # It might be better to start only slcan than with this script.
        # Node(
        #     package='slcan_cpp',
        #     executable='slcan',
        #     name='slcan',
        #     output={
        #         # 'stdout': 'screen',
        #         'stderr': 'screen',
        #         },
        #     emulate_tty=True,
        #     parameters=[
        #         {'slcan_topic_name': 'slcan'},
        #         {'serial_name': '/dev/ttyACM0',},
        #         {'baudrate', '115200'}
        #     ]
        # ),
        
    ])