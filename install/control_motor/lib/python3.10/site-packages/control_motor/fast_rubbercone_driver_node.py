#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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

    def __init__(self):
        super().__init__('rubbercone_driver')

        # 1) (튜닝용) 파라미터 선언만 유지 — 주행/클러스터링 관련 값들
        declared = self.declare_parameters(
            '',
            [
                ('min_group_points', 5),
                ('max_group_distance', 0.05),
                ('distance_proportion', 0.00628),  # 현재 미사용(확장용)
                ('max_circle_radius', 0.1),

                ('min_x_limit', 0.0),
                ('max_x_limit', 0.6),
                ('min_y_limit', -0.55),
                ('max_y_limit', 0.9),

                ('frame_id', 'laser_frame'),

                ('lookahead', 0.6),
                ('wheelbase', 0.26),              # 현재 미사용(확장용)
                ('max_steer_deg', 80.0),

                ('base_speed', 12.0),
                ('min_speed', 8.0),
                ('max_speed', 20.0),

                ('lateral_slowdown_gain', 5.0),   # 속도 고정으로 현재 미사용
                ('yaw_gain', 3.0),

                # 색 게이트 안정화 윈도우(여긴 파라미터 그대로 둠)
                ('orange_hold_frames', 5),

                # ===== [추가] 조향 안정화 파라미터 =====
                ('steer_alpha', 0.5),          # 저역통과 필터 계수(0.0~1.0)
                ('steer_rate_limit', 300.0),    # 레이트 리밋 [deg/s]
                ('steer_deadband_deg', 0.1),    # 데드밴드 [deg]
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

        # ===== [추가] 조향 필터 설정 =====
        self.steer_alpha        = float(g['steer_alpha'])
        self.steer_rate_limit   = float(g['steer_rate_limit'])
        self.steer_deadband_deg = float(g['steer_deadband_deg'])
        self._prev_steer_deg   = 0.0
        self._last_steer_stamp = self.get_clock().now()

        # 3) QoS / Pub-Sub (고정 토픽 사용)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.create_subscription(LaserScan, self.SCAN_TOPIC, self.on_scan, qos)

        self.activate_on_orange = self.ACTIVATE_ON_ORANGE
        if self.activate_on_orange:
            self.create_subscription(Bool, self.IS_ORANGE_TOPIC, self.on_orange, 10)

        # 퍼블리셔: 항상 [angle, speed, flag]
        self.pub = self.create_publisher(Float32MultiArray, self.OUT_TOPIC, 10)

        # RViz 마커 퍼블리셔
        self.visualize = self.VISUALIZE
        self.marker_pub = self.create_publisher(MarkerArray, '/rubbercone/markers', 10)
        self.marker_lifetime = Duration(seconds=0.25)

        # 색 게이트 상태 메모리
        self._orange_state = False
        self._orange_deque = deque(maxlen=self.orange_hold_frames)

        self.get_logger().info(
            f"rubbercone_driver subscribed to {self.SCAN_TOPIC}, publishing {self.OUT_TOPIC} "
            f"(gate={'ON' if self.activate_on_orange else 'OFF'}) [Float32MultiArray: [angle, speed, flag]]"
        )

    # -------------------- Callbacks --------------------
    def on_orange(self, msg: Bool):
        self._orange_deque.append(bool(msg.data))
        # 다수결(60% 이상 True)로 안정화
        self._orange_state = (sum(self._orange_deque) >= int(len(self._orange_deque) * 0.6))

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
        #self.get_logger().info(f"left : {l}")
        #self.get_logger().info(f"right : {r}")

        # 6) 중앙/목표점 설정
        if l is not None and r is not None:
            mid = 0.5 * (l + r)
            print("@@@@@@@@@@@@")
        elif l is not None:
            mid = l + np.array([0.0, -0.4])   # 좌만 보이면 오른쪽으로
        else:
            mid = r + np.array([0.0,  0.4])   # 우만 보이면 왼쪽으로
            self.get_logger().info(f"!!!!!!!!!!!!!")

        target = np.array([min(self.max_x, mid[0] + self.lookahead), mid[1]])

        # 7) 조향/속도 계산
        steer_raw = self._steer_from_target(target)
        steer     = self._apply_steer_filters(steer_raw)   # ===== [추가] 조향 필터 적용
        speed     = self._speed_from_target(target, steer) # ===== [수정] 일정 속도

        # flag: 게이트 통과 && 타깃 생성 시 True
        gate_ok = (not self.activate_on_orange) or self._orange_state
        self._publish(-steer, speed, bool(gate_ok))  # 기존 부호 유지

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

    # ===== [수정] 속도는 항상 base_speed로 고정 =====
    def _speed_from_target(self, t, steer_deg):
        # 요청대로 일정 속도 유지 (게이트 미통과 시 on_scan에서 0으로 정지)
        return float(max(self.min_speed, min(self.max_speed, self.base_speed)))

    # ===== [추가] 조향 레이트 리밋 + 저역통과 + 데드밴드 =====
    def _apply_steer_filters(self, steer_deg: float) -> float:
        # 0) 데드밴드
        if abs(steer_deg) < self.steer_deadband_deg:
            steer_deg = 0.0

        # 1) 레이트 리밋(초당 변화량 제한)
        now = self.get_clock().now()
        dt  = max(1e-3, (now - self._last_steer_stamp).nanoseconds * 1e-9)  # 최소 1ms 가드
        max_delta = self.steer_rate_limit * dt
        delta     = steer_deg - self._prev_steer_deg
        if abs(delta) > max_delta:
            steer_deg = self._prev_steer_deg + math.copysign(max_delta, delta)

        # 2) 1차 저역통과(IIR)
        alpha     = min(max(self.steer_alpha, 0.0), 1.0)
        filtered  = self._prev_steer_deg + alpha * (steer_deg - self._prev_steer_deg)

        # 3) 포화
        filtered  = max(-self.max_steer_deg, min(self.max_steer_deg, filtered))

        # 상태 업데이트
        self._prev_steer_deg   = filtered
        self._last_steer_stamp = now
        return filtered

    def _publish(self, angle, speed, flag):
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


if __name__ == '__main__':
    main()
