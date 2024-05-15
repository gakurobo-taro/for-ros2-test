from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
            Node(
                package='uart_communicator',
                executable='main',
                exec_name='uart_communicator',
                name='uart_communicator',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {"serial_name": "/dev/ttyACM1"}
                ]
            ),
        Node(
            package='imu_reader',
            executable='publisher',
            name='publisher',
            output={
                # 'stdout': 'screen',
                'stderr': 'screen',
                },
            emulate_tty=True,
        ),
        Node(
            package='imu_estimate',
            executable='imu',
            output={
                # 'stdout': 'screen',
                'stderr': 'screen',
                },
            emulate_tty=True,
        ),
        Node(package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0.185', '0', '0.0','0', '0', '0', '1','base_footprint','imu0'],
        )
])