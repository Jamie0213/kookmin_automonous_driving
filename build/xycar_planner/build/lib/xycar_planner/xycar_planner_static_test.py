#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# 메시지 타입 임포트 경로가 ROS2에 맞게 수정되었습니다.
# xycar_msgs 패키지가 ROS2 환경에 맞게 빌드되어 있다고 가정합니다.
from xycar_msgs.msg import XycarMotor
from std_msgs.msg import Int64, String
# obstacle_detector 패키지가 ROS2 환경에 맞게 빌드되어 있다고 가정합니다.
from obstacle_msgs.msg import Waypoint, CarObstacles
from rclpy.qos import QoSProfile

from math import degrees, atan2
import numpy as np
import math
# import cv2 # 원본 코드에 있으나 실제 사용되지 않아 주석 처리
# from cv_bridge import CvBridge, CvBridgeError # 원본 코드에 있으나 실제 사용되지 않아 주석 처리
import time

# 장애물 정보를 저장하기 위한 클래스
class Obstacle:
    def __init__(self, x=None, y=None, distance=None):
        self.x = x
        self.y = y
        self.distance = distance

# PID 제어기 클래스
class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def pid_control(self, cte):
        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte
        return self.kp * self.p_error + self.ki * self.i_error + self.kd * self.d_error

# Xycar의 주행 계획 및 제어를 담당하는 메인 클래스 (ROS2 Node 클래스로 변경)
class XycarPlanner(Node):
    def __init__(self):
        # 노드 초기화. 노드 이름은 'xycar_planner_humble'로 지정
        super().__init__('xycar_planner_humble')
        self.qos_profile = QoSProfile(depth=1)

        # ROS2 방식으로 퍼블리셔와 서브스크라이버를 생성합니다.
        # QoS 프로파일의 깊이는 10으로 설정하는 것이 일반적입니다.
        self.ctrl_cmd_pub = self.create_publisher(XycarMotor, '/xycar_motor', self.qos_profile)
        self.mode_pub = self.create_publisher(String, '/mode', 10)

        # 서브스크라이버의 메시지 타입을 XycarMotor로 통일합니다.
        self.create_subscription(XycarMotor, "/xycar_motor_lane", self.ctrlLaneCB, 10)
        self.create_subscription(XycarMotor, "/xycar_motor_static", self.ctrlStaticCB, 10)
        self.create_subscription(Int64, "/traffic_light", self.trafficLightCB, 10)
        self.create_subscription(CarObstacles, "/raw_obstacles_static", self.obstacleCB, 10)
        self.create_subscription(Waypoint, '/rubbercone_waypoints', self.ctrlRubberconeCB, 10)

        # self.bridge = CvBridge() # cv_bridge 사용되지 않아 주석 처리

        # 주행 제어 관련 변수 초기화 (이전과 동일)
        self.steer = 0.0
        self.motor = 0.0
        self.traffic_light = 0  # 1: 정지, 2: 직진
        self.ctrl_cmd_msg = XycarMotor()

        # 각 모드별 제어 명령 저장 변수 (메시지 타입을 XycarMotor로 통일)
        self.ctrl_cmd = XycarMotor()
        self.ctrl_lane = XycarMotor()
        self.ctrl_static = XycarMotor()
        self.ctrl_rubbercone = XycarMotor()

        # 주행 모드 플래그
        self.static_mode_flag = False
        self.lane_mode_flag = False

        # PID 제어기 인스턴스 생성 (이전과 동일, 여전히 직접 사용되지는 않음)
        self.pid = PID(0.7, 0.0008, 0.15)
        self.obstacles = []

        # 라바콘 회피 모드 관련 변수 초기화 (이전과 동일)
        self.rubber_mode = False
        self.detected_frames = 0
        self.lost_frames = 0
        self.prev_wp_cnt = 0
        self.wp_cnt = 0
        self.latest_wp = None
        self.target_wp = None
        self.rubber_start_time = None
        self.no_points_frames = 0

        # ROS2 방식으로 파라미터를 선언하고 값을 가져옵니다.
        self.declare_parameter('rcon_speed', 10.0)
        self.rcon_speed = self.get_parameter('rcon_speed').get_parameter_value().double_value
        
        # ROS1의 rospy.get_param()에 해당되는 부분들은 위와 같이 변경합니다.
        self.entry_thresh = 3
        self.exit_thresh = 5
        self.reach_dist = 0.3
        self.max_duration = 30.0

        # ROS1의 `while` 루프 대신 30Hz로 주기적인 타이머 콜백을 사용합니다.
        self.timer_period = 1.0 / 30.0  # 30Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info('Xycar Planner Node has been initialized.')

    # ROS1의 메인 루프에 해당되던 로직을 타이머 콜백 함수로 이동
    def timer_callback(self):
        current_wp_available = (self.wp_cnt >= 1 and self.latest_wp is not None)

        # 라바콘 모드 진입/유지/이탈 로직
        if not self.rubber_mode:
            if current_wp_available:
                self.detected_frames += 1
            else:
                self.detected_frames = 0
            
            if self.detected_frames >= self.entry_thresh:
                self.rubber_mode = True
                # rospy.Time.now() -> self.get_clock().now()
                self.rubber_start_time = self.get_clock().now()
                self.target_wp = self.latest_wp
                self.lost_frames = 0
                self.get_logger().info('=== Enter Rubbercone Mode ===')

        else:  # 라바콘 모드일 경우
            if current_wp_available:
                self.target_wp = self.latest_wp
                self.lost_frames = 0
            else:
                self.lost_frames += 1
                # rospy.loginfo -> self.get_logger().info
                self.get_logger().info(f"Rubbercone Mode: No new waypoints, lost_frames = {self.lost_frames}/{self.exit_thresh}")

            if self.lost_frames >= self.exit_thresh:
                self.get_logger().info(f"=== Exit Rubbercone Mode (Lost Points: {self.lost_frames} frames) ===")
                self.rubber_mode = False
                self.lost_frames = 0
                self.detected_frames = 0
                self.target_wp = None
                self.rubber_start_time = None
                
                # 라바콘 모드 종료 후 정적 장애물 회피 모드로 전환
                # 이 부분은 time.sleep을 사용하여 ROS2의 실행 모델에 영향을 줄 수 있으므로 주의가 필요합니다.
                # 더 나은 방법은 상태 머신을 통해 전환하는 것이지만, 원본 로직을 최대한 유지했습니다.
                speed = 30.0
                steer = 48.0
                mode_str = String()
                mode_str.data = "StaticObs"
                
                # 일정 시간 동안 제어 명령을 유지하는 부분. rate.sleep() 대신 time.sleep() 사용.
                # 이 블록은 콜백 함수의 실행을 지연시키므로 긴 시간 동안 사용하는 것은 권장되지 않습니다.
                for _ in range(33):
                    self.ctrl_cmd.speed = speed
                    self.ctrl_cmd.angle = steer
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd)
                    self.mode_pub.publish(mode_str)
                    time.sleep(self.timer_period) # rate.sleep()과 유사하게 동작하도록 설정
                return # 현재 콜백을 종료하고 다음 콜백에서 다시 시작

        # 현재 주행 모드에 따른 속도 및 조향각 결정
        mode_msg = String()
        if self.rubber_mode and self.target_wp:
            x, y = self.target_wp
            ang = 3.9 * degrees(atan2(y, x))
            speed, steer = float(self.rcon_speed), -float(ang)
            mode_msg.data = "Rubbercone"
        elif self.static_mode_flag:
            speed = self.ctrl_static.speed
            steer = self.ctrl_static.angle
            mode_msg.data = "StaticObs"
        else:
            speed = self.ctrl_lane.speed
            steer = self.ctrl_lane.angle
            mode_msg.data = "LaneFollowing"

        # 신호등 상태에 따른 속도 제어
        if self.traffic_light == 1:
            speed = 20.0
        elif self.traffic_light == 2:
            pass

        # 최종 제어 명령 설정 및 발행
        self.ctrl_cmd.speed = float(speed)
        self.ctrl_cmd.angle = float(steer)
        self.ctrl_cmd_pub.publish(self.ctrl_cmd)
        self.mode_pub.publish(mode_msg)

    # 차선 주행 제어 명령 콜백 함수 (메시지 타입을 XycarMotor로 통일)
    def ctrlLaneCB(self, msg: XycarMotor):
        self.ctrl_lane.speed = msg.speed
        self.ctrl_lane.angle = msg.angle

    # 정적 장애물 회피 제어 명령 콜백 함수 (메시지 타입을 XycarMotor로 통일)
    def ctrlStaticCB(self, msg: XycarMotor):
        self.ctrl_static.speed = msg.speed
        self.ctrl_static.angle = msg.angle
        self.static_mode_flag = msg.flag

    # 신호등 상태 콜백 함수 (변경 없음)
    def trafficLightCB(self, msg: Int64):
        self.traffic_light = msg.data

    # 라바콘 웨이포인트 콜백 함수 (변경 없음)
    def ctrlRubberconeCB(self, msg: Waypoint):
        self.wp_cnt = msg.cnt
        if msg.cnt >= 1 and len(msg.x_arr) > 0 and len(msg.y_arr) > 0:
            self.latest_wp = (msg.x_arr[0], msg.y_arr[0])
        else:
            self.latest_wp = None

    # 정적 장애물 정보 콜백 함수 (변경 없음)
    def obstacleCB(self, msg: CarObstacles):
        self.obstacles = []
        for circle in msg.circles:
            x = circle.center.x
            y = circle.center.y
            distance = (x**2 + y**2) ** 0.5
            obstacle = Obstacle(x, y, distance)
            self.obstacles.append(obstacle)
        
        self.obstacles.sort(key=lambda obs: obs.distance)

        # 가장 가까운 장애물 정보 저장은 원본에 없었지만 유용한 로직이라 추가합니다.
        # 필요 없다면 이 부분을 삭제하셔도 됩니다.
        if len(self.obstacles) > 0:
            self.closest_obstacle = self.obstacles[0]
        else:
            self.closest_obstacle = Obstacle()


def main(args=None):
    # ROS2 노드 실행을 위한 표준적인 main 함수
    rclpy.init(args=args)
    
    node = XycarPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        # 노드 소멸 및 rclpy 종료
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
