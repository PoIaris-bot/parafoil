from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='parafoil',
            executable='uwb_publisher',
            name='uwb_publisher',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud_rate': 921600
            }]
        ),
        Node(
            package='parafoil',
            executable='rc_receiver',
            name='rc_receiver',
            parameters=[{
                'port': '/dev/ttyAMA0',
            }]
        ),
        Node(
            package='parafoil',
            executable='motor_driver',
            name='motor_driver',
            parameters=[{
                'motor_pin': 18,
                'left_servo_pin': 23,
                'right_servo_pin': 24
            }]
        ),
        Node(
            package='parafoil',
            executable='radio_telemetry',
            name='radio_telemetry',
            parameters=[{
                'topic': 'parafoil/pose/ekf',
                'rate': 15,
                'port': '/dev/ttyUSB1',
                'baud_rate': 57600
            }]
        ),
        Node(
            package='parafoil',
            executable='pose_estimator',
            name='pose_estimator'
        ),
        Node(
            package='parafoil',
            executable='autopilot',
            name='autopilot',
            parameters=[{
                'control_rate': 20,
            }]
        ),
        Node(
            package='parafoil',
            executable='logger',
            name='logger',
            parameters=[{
                'filepath': '/home/pi/ros2_ws/src/parafoil/log',
            }],
            output='screen'
        )
    ])
