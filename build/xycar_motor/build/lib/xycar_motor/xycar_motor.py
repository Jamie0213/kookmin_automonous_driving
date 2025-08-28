#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.clock import Clock
import time
import sys
import os
import signal
from xycar_msgs.msg import XycarMotor
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped

'''
def signal_handler(sig, frame):
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
'''

class Motor(Node):

    def __init__(self):
        super().__init__('xycar_motor')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('motor_type', 0),
                ('steering_angle_to_servo_offset', 0.0),
                ('speed_max', 0.0),
                ('speed_min', 0.0),
                ('speed_to_erpm_gain', 0.0),
            ])
        
        self.qos_profile = QoSProfile(depth=1)
        self.ros_init()
        self.set_vesc()

        # 파라미터 로드
        self.angle_offset = self.get_parameter('steering_angle_to_servo_offset').value
        self.speed_max = self.get_parameter('speed_max').value
        self.speed_min = self.get_parameter('speed_min').value
        self.speed_to_erpm_gain = self.get_parameter('speed_to_erpm_gain').value
        self.vesc_smax = self.speed_max / self.speed_to_erpm_gain
        self.vesc_smin = self.speed_min / self.speed_to_erpm_gain

        '''
        self.get_logger().info(f"angle_offset: {self.angle_offset:.4f}")
        self.get_logger().info(f"speed_max: {self.speed_max:.2f}")
        self.get_logger().info(f"speed_min: {self.speed_min:.2f}")
        self.get_logger().info(f"speed_to_erpm_gain: {self.speed_to_erpm_gain:.4f}")
        self.get_logger().info(f"vesc_smax: {self.vesc_smax:.4f}")
        self.get_logger().info(f"vesc_smin: {self.vesc_smin:.4f}")
        '''
                    
        # 속도 제어 변수
        self.last_speed = 0.0
        self.change_vector_term = 0.07  # 속도 변화 간격 (초)

        self.count = 0

    def set_vesc(self):
    
        # VESC 초기화 대기
        self.get_logger().info("VESC 초기화 중... 3초 대기")
        time.sleep(1)
       
        """VESC 상태 정보를 받아오는 Subscriber 설정 및 초기화 대기"""
        self.sensors_core_subscriber = self.create_subscription(
            VescStateStamped,
            '/sensors/core',
            self.VescStateCallback,
            self.qos_profile)

        time.sleep(1)
        self.get_logger().info("VESC 연결 완료")

    def VescStateCallback(self, data):
        """VESC 상태 메시지 콜백 함수"""
        battery_voltage = data.state.voltage_input
        fault_code = data.state.fault_code
        self.count = self.count+1
        
        if fault_code:
            self.get_logger().fatal(f"VESC Fault: {fault_code} -- {self.count}")
            self.get_logger().fatal(f"Battery Voltage: {battery_voltage}")
            time.sleep(0.1)

    def ros_init(self):
        """ROS2 노드 초기화"""
        self.motor_subscriber = self.create_subscription(
            XycarMotor,
            'xycar_motor',
            self.callback,
            self.qos_profile)

        self.ackermann_publisher = self.create_publisher(
            AckermannDriveStamped,
            'ackermann_cmd',
            self.qos_profile)

    def auto_drive_vesc(self, steer_val, car_run_speed):
        """VESC를 통한 차량 속도 및 스티어링 제어"""
        msg = AckermannDriveStamped()
        msg.header.stamp = Clock().now().to_msg()
        
        # 스티어링 및 속도 설정
        msg.drive.steering_angle = steer_val  # 변환된 스티어링 값 적용
        # msg.drive.steering_angle = msg.drive.steering_angle + self.angle_offset
                
        msg.drive.speed = car_run_speed  # 변환된 속도 값 적용

        # 감속 처리 (속도 변화 완화)
        cnt = 10
        min_v_piece = self.vesc_smax / (2.0 * cnt)

        '''
        if ((self.last_speed > 0 and car_run_speed <= 0) or 
            (self.last_speed < 0 and car_run_speed >= 0)):
            reduce_v = max(abs(car_run_speed - self.last_speed) / cnt, min_v_piece)
            vector = -1 if self.last_speed > 0 else 1

            for c in range(cnt):
                intermediate_speed = self.last_speed + (vector * c * reduce_v)
                msg.drive.speed = max(min(intermediate_speed, car_run_speed), self.vesc_smin)
                self.ackermann_publisher.publish(msg)
                time.sleep(self.change_vector_term)
        '''
        
        msg.drive.speed = car_run_speed
        self.ackermann_publisher.publish(msg)
        self.last_speed = car_run_speed

    def callback(self, data):
        """XycarMotor 메시지 콜백 함수"""
        angle = max(-100.0, min(float(data.angle), 100.0))  # -100.0 ~ 100.0 범위로 제한
        speed = max(-100.0, min(float(data.speed), 100.0))  # -100.0 ~ 100.0 범위로 제한

        # 조향각 변환: -100.0 ~ 100.0 → -0.3 ~ 0.3
        steer_val = -(angle / 100.0) * 0.3
        
        # 속도 변환: -100.0 ~ 100.0 → -1.8 ~ 1.8
        speed_val = (speed / 100.0) * 5.0

        if abs(speed_val) < 0.5:
            speed_val = 0.0
            
        self.auto_drive_vesc(steer_val, speed_val)

def main(args=None):
    """ROS2 노드 실행"""
    rclpy.init(args=args)
    node = Motor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass            

if __name__ == '__main__':
    main()

