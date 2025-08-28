#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.duration import Duration


class RubberconeDriver(Node):
    # 고정 설정(런치에서 넘기지 않음)
    SCAN_TOPIC = '/scan'
    OUT_TOPIC = '/rubbercone'
    ACTIVATE_ON_ORANGE = True
    VISUALIZE = True
    IS_ORANGE_TOPIC = '/is_orange'   # 색 게이트 입력 토픽
    RACE_STARTED_TOPIC = '/race_started'

    def __init__(self):
        super().__init__('rubbercone_driver')

        # 1) (튜닝용) 파라미터 선언 — 기존 값 유지 + race_flag_off_after_sec 추가
        declared = self.declare_parameters(
            '',
            [
                ('min_group_points', 5),
                ('max_group_distance', 0.05),
                ('distance_proportion', 0.00628),  # 현재 미사용(확장용)
                ('max_circle_radius', 0.1),

                ('min_x_limit', 0.0),
                ('max_x_limit', 0.6),
                ('min_y_limit', -0.65),
                ('max_y_limit', 0.9),

                ('frame_id', 'laser_frame'),

                ('lookahead', 0.6),
                ('wheelbase', 0.26),              # 현재 미사용(확장용)
                ('max_steer_deg', 80.0),

                ('base_speed', 27.5),
                ('min_speed', 22.5),
                ('max_speed', 35.0),

                ('lateral_slowdown_gain', 5.0),
                ('yaw_gain', 4.4),

                # 색 게이트 안정화 윈도우(여긴 파라미터 그대로 둠)
                ('orange_hold_frames', 5),

                # ★ 추가: 신호등 출발 후 flag=0 강제 전환 지연(sec)
                ('race_flag_off_after_sec', 25.0),
            ]
        )
        g = {p.name: p.value for p in declared}

        # 2) 형 변환하여 멤버 저장
        self.min_group_points      = int(g['min_group_points'])
        self.max_group_distance    = float(g['max_group_distance'])
        self.distance_proportion   = float(g['distance_proportion'])
        self.max_circle_radius     = float(g['max_circle_radius'])

        self.min_x                 = float(g['min_x_limit'])
        self.max_x                 = float(g['max_x_limit'])
        self.min_y                 = float(g['min_y_limit'])
        self.max_y                 = float(g['max_y_limit'])

        self.frame_id              = str(g['frame_id'])

        self.lookahead             = float(g['lookahead'])
        self.wheelbase             = float(g['wheelbase'])
        self.max_steer_deg         = float(g['max_steer_deg'])

        self.base_speed            = float(g['base_speed'])
        self.min_speed             = float(g['min_speed'])
        self.max_speed             = float(g['max_speed'])

        self.lateral_slowdown_gain = float(g['lateral_slowdown_gain'])
        self.yaw_gain              = float(g['yaw_gain'])

        self.orange_hold_frames    = max(1, int(g['orange_hold_frames']))

        # ★ 추가: 출발 후 flag 강제 오프 지연(s)
        self.race_flag_off_after_sec = float(g['race_flag_off_after_sec'])

        # 3) QoS / Pub-Sub (고정 토픽 사용)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(LaserScan, self.SCAN_TOPIC, self.on_scan, qos)

        self.activate_on_orange = self.ACTIVATE_ON_ORANGE
        if self.activate_on_orange:
            self.create_subscription(Bool, self.IS_ORANGE_TOPIC, self.on_orange, 10)

        # ★ 추가: /race_started 구독 (latched 수신을 위해 TRANSIENT_LOCAL)
        start_qos = QoSProfile(depth=1)
        start_qos.reliability = ReliabilityPolicy.RELIABLE
        start_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        start_qos.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(Bool, self.RACE_STARTED_TOPIC, self.on_race_started, start_qos)

        # 퍼블리셔: 항상 [angle, speed, flag]
        self.pub = self.create_publisher(Float32MultiArray, self.OUT_TOPIC, 10)

        # RViz 마커 퍼블리셔
        self.visualize = self.VISUALIZE
        self.marker_pub = self.create_publisher(MarkerArray, '/rubbercone/markers', 10)
        self.marker_lifetime = Duration(seconds=0.25)

        # 색 게이트 상태 메모리
        self._orange_state = False
        self._orange_deque = deque(maxlen=self.orange_hold_frames)

        # ★ 추가 상태: 출발 후 강제 flag=0 스위치 & 타이머
        self._force_flag_off = False
        self._flag_off_timer = None  # one-shot timer 핸들

        self.get_logger().info(
            f"rubbercone_driver subscribed to {self.SCAN_TOPIC}, publishing {self.OUT_TOPIC} "
            f"(gate={'ON' if self.activate_on_orange else 'OFF'}) [Float32MultiArray: [angle, speed, flag]]"
        )

    # -------------------- Callbacks --------------------
    def on_orange(self, msg: Bool):
        self._orange_deque.append(bool(msg.data))
        # 다수결(60% 이상 True)로 안정화
        self._orange_state = (sum(self._orange_deque) >= int(len(self._orange_deque) * 0.6))

    # ★ 추가: /race_started 콜백 — True 수신 시, 15초 뒤 flag 강제 OFF
    def on_race_started(self, msg: Bool):
        if bool(msg.data) and (self._flag_off_timer is None) and (not self._force_flag_off):
            self.get_logger().info(
                f"[/race_started] received → will force cone flag=0 after {self.race_flag_off_after_sec:.1f}s"
            )
            # one-shot timer
            self._flag_off_timer = self.create_timer(self.race_flag_off_after_sec, self._enable_force_flag_off)

    # ★ 추가: 타이머 핸들러 (한 번만 실행)
    def _enable_force_flag_off(self):
        self._force_flag_off = True
        self.get_logger().info("Rubbercone: flag is now permanently forced to 0 (cone gating disabled).")
        if self._flag_off_timer is not None:
            self._flag_off_timer.cancel()
            self._flag_off_timer = None

    def on_scan(self, scan: LaserScan):
        # 색 게이트가 켜져 있고(_orange_state=False)이면 즉시 정지/flag=False
        if self.activate_on_orange and not self._orange_state:
            self._publish(0.0, 0.0, False)
            if self.visualize:
                self._publish_markers([], None, None, None, None, [], [], [])
            return

        # 1) LaserScan -> XY
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        ranges = np.array(scan.ranges, dtype=np.float32)
        valid = np.isfinite(ranges)
        ranges = ranges[valid]; angles = angles[valid]
        xs = ranges * np.cos(angles); ys = ranges * np.sin(angles)

        # 2) ROI 필터
        m = (xs >= self.min_x) & (xs <= self.max_x) & (ys >= self.min_y) & (ys <= self.max_y)
        xs, ys = xs[m], ys[m]
        if xs.size == 0:
            self._publish(0.0, 0.0, False)
            if self.visualize:
                self._publish_markers([], None, None, None, None, [], [], [])
            return

        pts = np.stack([xs, ys], axis=1)

        # 3) 유클리드 클러스터링
        clusters = self._euclidean_clusters(pts)

        # 4) 라바콘 후보 선별 (작고 둥근 클러스터)
        cones = []
        for c in clusters:
            if c.shape[0] < self.min_group_points:
                continue
            center = np.mean(c, axis=0)
            r = float(np.sqrt(np.mean(np.sum((c - center) ** 2, axis=1))))  # RMS 반경
            if r <= self.max_circle_radius:
                cones.append((center, r))

        if not cones:
            self._publish(0.0, 0.0, False)
            if self.visualize:
                self._publish_markers(pts, None, None, None, None, [], [], [])
            return

        # 5) 좌/우 분리 & Pivot 선택 (가장 전방 x)
        left  = [c for c in cones if c[0][1] > 0.0]
        right = [c for c in cones if c[0][1] < 0.0]
        if not left and not right:
            self._publish(0.0, 0.0, False)
            if self.visualize:
                self._publish_markers(pts, None, None, None, None, left, right, cones)
            return

        def pick(side):
            if not side:
                return None
            side.sort(key=lambda cr: (cr[0][0], -abs(cr[0][1])), reverse=True)
            return side[0][0]

        l = pick(left); r = pick(right)

        # 6) 중앙/목표점 설정 (기존 그대로)
        if l is not None and r is not None:
            mid = 0.5 * (l + r)
        elif l is not None:
            mid = l + np.array([0.0, -0.4])   # 좌만 보이면 오른쪽으로 0.4m
        else:
            mid = r + np.array([0.0,  0.4])   # 우만 보이면 왼쪽으로 0.4m
            #self.get_logger().info(f"!!!!!!!!!!!!!")

        target = np.array([min(self.max_x, mid[0] + self.lookahead), mid[1]])

        # 7) 조향/속도 계산 (기존 그대로)
        steer = self._steer_from_target(target)
        speed = self._speed_from_target(target, steer)

        # flag: 게이트 통과 && 타깃 생성 시 True (※ 최종 퍼블리시에서 강제 오프 적용)
        gate_ok = (not self.activate_on_orange) or self._orange_state
        self._publish(-steer, speed, bool(gate_ok))

        # RViz 시각화 퍼블리시
        if self.visualize:
            self._publish_markers(pts, l, r, mid, target, left, right, cones)

    # -------------------- Helpers --------------------
    def _euclidean_clusters(self, pts: np.ndarray):
        """간단한 grid binning + DFS 기반 유클리드 클러스터링."""
        thr = self.max_group_distance
        n = pts.shape[0]
        if n == 0:
            return []
        visited = np.zeros(n, dtype=bool)
        clusters = []
        inv = 1.0 / max(1e-6, thr)
        grid = {}
        for i, (x, y) in enumerate(pts):
            gx, gy = int(x * inv), int(y * inv)
            grid.setdefault((gx, gy), []).append(i)

        def neighbors(i):
            x, y = pts[i]
            gx, gy = int(x * inv), int(y * inv)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for j in grid.get((gx + dx, gy + dy), []):
                        if j == i or visited[j]:
                            continue
                        if np.hypot(*(pts[j] - pts[i])) <= thr:
                            yield j

        for i in range(n):
            if visited[i]:
                continue
            stack = [i]; visited[i] = True; comp = [i]
            while stack:
                k = stack.pop()
                for j in neighbors(k):
                    visited[j] = True
                    stack.append(j)
                    comp.append(j)
            clusters.append(pts[np.array(comp)])
        return clusters

    def _steer_from_target(self, t):
        yaw_err = math.atan2(float(t[1]), float(t[0]))  # 라디안
        steer_deg = math.degrees(self.yaw_gain * yaw_err)
        return max(-self.max_steer_deg, min(self.max_steer_deg, steer_deg))

    def _speed_from_target(self, t, steer_deg):
        lateral = abs(float(t[1]))
        speed = self.base_speed - self.lateral_slowdown_gain * lateral - 0.2 * abs(steer_deg)
        return float(max(self.min_speed, min(self.max_speed, speed)))

    def _publish(self, angle, speed, flag):
        """최종 퍼블리시 직전에 race-start 강제off를 적용."""
        if self._force_flag_off:
            flag = False
        msg = Float32MultiArray()
        msg.data = [float(angle), float(speed), 1.0 if flag else 0.0]
        self.pub.publish(msg)

    # -------------------- RViz Markers --------------------
    def _publish_markers(self, pts, l, r, mid, target, left_cones, right_cones, cones_all):
        arr = MarkerArray()

        # 이전 마커 정리
        del_all = Marker()
        del_all.header.frame_id = self.frame_id
        del_all.header.stamp = self.get_clock().now().to_msg()
        del_all.ns = "rubbercone"
        del_all.id = 0
        del_all.action = Marker.DELETEALL
        arr.markers.append(del_all)

        stamp = self.get_clock().now().to_msg()
        lifetime = self.marker_lifetime.to_msg()

        # ROI 포인트
        if len(pts) > 0:
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns = "rubbercone"
            m.id = 1
            m.type = Marker.POINTS
            m.action = Marker.ADD
            m.scale.x = 0.02
            m.scale.y = 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 0.8, 0.8, 0.8
            m.lifetime = lifetime
            m.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in pts]
            arr.markers.append(m)

        # 좌/우 라바콘(센터들)
        def add_sphere_list(marker_id, cone_list, rgba):
            if not cone_list:
                return
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns = "rubbercone"
            m.id = marker_id
            m.type = Marker.SPHERE_LIST
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            m.lifetime = lifetime
            m.points = [Point(x=float(c[0][0]), y=float(c[0][1]), z=0.0) for c in cone_list]
            arr.markers.append(m)

        add_sphere_list(2, left_cones,  (0.2, 0.8, 0.2, 0.9))
        add_sphere_list(3, right_cones, (0.2, 0.2, 0.8, 0.9))

        # Pivot / Mid / Target 표시
        def add_sphere(marker_id, xy, rgba, scale=0.12):
            if xy is None:
                return
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns = "rubbercone"
            m.id = marker_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            m.pose.position.x = float(xy[0])
            m.pose.position.y = float(xy[1])
            m.pose.position.z = 0.0
            m.lifetime = lifetime
            arr.markers.append(m)

        add_sphere(10, l,      (0.1, 1.0, 0.1, 0.95))
        add_sphere(11, r,      (0.1, 0.6, 1.0, 0.95))
        add_sphere(12, mid,    (1.0, 1.0, 0.1, 0.95))
        add_sphere(13, target, (1.0, 0.2, 0.2, 0.95))

        # 타깃 가이드 라인
        if target is not None:
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns = "rubbercone"
            m.id = 14
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.3, 0.3, 0.9
            m.points = [Point(x=0.0, y=0.0, z=0.0),
                        Point(x=float(target[0]), y=float(target[1]), z=0.0)]
            m.lifetime = lifetime
            arr.markers.append(m)

        # ROI 경계
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = "rubbercone"
        m.id = 15
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.015
        m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.9, 0.1, 0.8
        m.lifetime = lifetime
        rect = [
            (self.min_x, self.min_y),
            (self.max_x, self.min_y),
            (self.max_x, self.max_y),
            (self.min_x, self.max_y),
            (self.min_x, self.min_y),
        ]
        m.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in rect]
        arr.markers.append(m)

        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = RubberconeDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
