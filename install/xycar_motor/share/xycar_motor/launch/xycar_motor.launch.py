from launch import LaunchDescription
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    vesc_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vesc_driver'),
                'launch/vesc_driver_node.launch.py'))
    )
    
    ackermann_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vesc_ackermann'),
                'launch/ackermann_to_vesc_node.launch.py'))
    )
        
    vesc_config = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config.yaml'
    )
    
    return LaunchDescription([
        vesc_include,
        ackermann_include,
        Node(
            package='xycar_motor',
            executable='xycar_motor',
            name='motor',
            parameters=[vesc_config, {
                'speed_to_erpm_gain': 4614.0,
                'speed_to_erpm_offset': 0.0,
                'steering_angle_to_servo_gain': -1.2135,
                'steering_angle_to_servo_offset': 0.3500
            }],
        ),
    ])
    


