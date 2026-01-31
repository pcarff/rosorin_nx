from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the joystick driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        ),
        # Start the control translator
        Node(
            package='peripherals',
            executable='joystick_control',
            name='joystick_control',
            parameters=[{
                'max_linear': 0.5,
                'max_angular': 2.0,
                'disable_servo_control': True
            }],
            # Remap the input topic to match joy_node's output
            remappings=[('ros_robot_controller/joy', 'joy')]
        )
    ])
