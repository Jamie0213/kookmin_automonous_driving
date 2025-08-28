# ==============================================================
# File: control_motor_main_with_heading.launch.py  (패치본)
# Desc: 사용자가 준 메인 런치에 imu_heading_node 만 추가. avoidance_controller_heading 는 포함하지 않음.
#       imu 드라이버(기존) + imu_heading_node → /heading 제공, 나머지 노드는 기존 그대로.
# Usage:
#   ros2 launch <패키지> control_motor_main_with_heading.launch.py
# ==============================================================

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    DEV = '/dev/video0'
    ctrl_pkg = 'control_motor'   # 기존 제어 패키지명 (필요시 수정)

    # 0) 카메라 시작 전: 원하는 값 선적용(고정값)
    pre_lock = ExecuteProcess(
        cmd=['bash','-lc', f'''
set -e
v4l2-ctl -d {DEV} --set-ctrl=white_balance_automatic=0
v4l2-ctl -d {DEV} --set-ctrl=white_balance_temperature=6000
v4l2-ctl -d {DEV} --set-ctrl=power_line_frequency=2,auto_exposure=1,exposure_time_absolute=160,gain=0
v4l2-ctl -d {DEV} --set-ctrl=backlight_compensation=0,hue=-10,saturation=55,gamma=105
'''],
        output='screen'
    )

    # 1) LiDAR 포함
    lidar_pkg = get_package_share_directory('xycar_lidar')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, 'launch', 'xycar_lidar.launch.py')),
    )

    # IMU 포함 (기존 드라이버 런치)
    imu_pkg = get_package_share_directory('xycar_imu')
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_pkg, 'launch', 'xycar_imu.launch.py')
        )
    )

    # 1.5) 헤딩 산출 노드 추가 (imu_heading_node)
    heading_node = Node(
        package=ctrl_pkg,   # imu_heading_node.py를 배치한 패키지명으로 수정
        executable='imu_heading_node',
        name='imu_heading',
        output='screen',
        parameters=[
            {'yaw_offset_deg': 0.0},
            {'heading_ema_alpha': 0.25},
            {'publish_diagnostics': True},
        ]
    )

    # 2) 카메라 노드 (대한민국 60Hz + 고정 파라미터)
    cam_node = Node(
        package='v4l2_camera', executable='v4l2_camera_node',
        name='cam_node', output='screen',
        parameters=[
            {'device': DEV},
            {'pixel_format': 'YUYV'},
            {'output_encoding': 'bgr8'},
            {'image_size': [640, 480]},
            {'frame_id': 'camera_frame'},

            {'white_balance_automatic': True},
            {'white_balance_temperature': 4800},
            {'auto_exposure': 1},
            {'exposure_time_absolute': 60},
            {'gain': 0},
            {'power_line_frequency': 2},
            {'backlight_compensation': 0},
            {'hue': -3},
            {'saturation': 55},
            {'gamma': 100},
        ],
        remappings=[('image_raw', '/image_raw')],
    )

    # 3) 카메라 시작 직후 재고정(초기화 경합 대비)
    lock_after_cam = RegisterEventHandler(
        OnProcessStart(
            target_action=cam_node,
            on_start=[ExecuteProcess(
                cmd=['bash','-lc', f'''
set -e
v4l2-ctl -d {DEV} --set-ctrl=white_balance_automatic=0
v4l2-ctl -d {DEV} --set-ctrl=white_balance_temperature=6000
v4l2-ctl -d {DEV} --set-ctrl=power_line_frequency=2,auto_exposure=1,exposure_time_absolute=160,gain=0
v4l2-ctl -d {DEV} --set-ctrl=backlight_compensation=0,hue=-10,saturation=55,gamma=105
'''],
                output='screen'
            )]
        )
    )

    # ───────────────────────────────────────────────────────────
    # VESC driver (하드웨어 I/O)
    vesc_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vesc_driver'),
                'launch', 'vesc_driver_node.launch.py'
            )
        )
    )

    # Ackermann → VESC 변환 노드
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        output='screen',
        parameters=[{
            'speed_to_erpm_gain': 4614.0,
            'speed_to_erpm_offset': 0.0,
            'steering_angle_to_servo_gain': -1.2135,
            'steering_angle_to_servo_offset': 0.3554,
        }]
    )

    vesc_config = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params', 'vesc_config.yaml'
    )
    xycar_motor_node_hw = Node(
        package='xycar_motor',
        executable='xycar_motor',
        name='xycar_motor',
        output='screen',
        parameters=[vesc_config, {
            'speed_to_erpm_gain': 4614.0,
            'speed_to_erpm_offset': 0.0,
            'steering_angle_to_servo_gain': -1.2135,
            'steering_angle_to_servo_offset': 0.3554,
        }]
    )

    # 4) 제어·알고리즘 노드들 (필요 시 수정)
    lane_detection_node   = Node(package=ctrl_pkg, executable='new_lane_detection',  name='lane_detection_node')
    motor_relay_node      = Node(package=ctrl_pkg, executable='kmu_motor_node',      name='motor_node')
    rubbercone_node       = Node(package=ctrl_pkg, executable='new_rubbercone_driver_node', name='rubbercone_node')
    trafficlight_node     = Node(package=ctrl_pkg, executable='trafficlight',        name='traffic_light_node')
    #obstacle_node         = Node(package=ctrl_pkg, executable='obstacle',            name='obstacle_node')
    collision_avoid_node  = Node(package=ctrl_pkg, executable='vehicle_avoidance',   name='collision_avoidance_node')
    is_orange_node        = Node(package=ctrl_pkg, executable='is_orange',           name='is_orange')
    is_vehicle_node       = Node(package=ctrl_pkg, executable='is_vehicle',          name='is_vehicle')

    return LaunchDescription([
        pre_lock,
        lidar_launch,
        cam_node,
        imu_launch,
        heading_node,     # ★ 추가: /heading 퍼블리셔
        lock_after_cam,
        vesc_driver_launch,
        xycar_motor_node_hw,
        ackermann_to_vesc_node,
        lane_detection_node,
        motor_relay_node,
        rubbercone_node,
        trafficlight_node,
        #obstacle_node,
        is_orange_node,
        collision_avoid_node,
        is_vehicle_node,
    ])
