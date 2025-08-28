from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Launch argument for debug flag
    debug_arg = DeclareLaunchArgument('debug', default_value='false', description='Run in debug mode (GDB)')

    # Launch prefix for GDB if debug is enabled
    launch_prefix = "xterm -e gdb --args"  # Only used if debug is true
    
    # Ackermann to VESC node
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        output='screen',
        parameters=[{
            'speed_to_erpm_gain': 4614.0,
            'speed_to_erpm_offset': 0.0,
            'steering_angle_to_servo_gain': -1.2135,
            'steering_angle_to_servo_offset': 0.3600  # 5304
        }],
        # Add GDB prefix if debug is true
        # If you need to handle debug mode, you'll need to adjust launch prefix logic
    )
    
    return LaunchDescription([
        debug_arg,
        ackermann_to_vesc_node
    ])

