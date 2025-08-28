#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 – VehicleDetector (ROI + 연속빔/점수 + chord gate + ROI 박스 라인 마커)
- Subscribe: /scan (sensor_msgs/LaserScan)
- Params (declare_parameter 로 런치/CLI에서 덮어쓰기 가능):
    · roi_y_abs    : 좌우 반폭 [m] (기본 0.4)
    · roi_x_min    : 전방 최소 [m] (기본 0.0)
    · roi_x_max    : 전방 최대 [m] (기본 2.0)
    · tau_r        : 인접빔 거리차 게이트 [m] (기본 0.15)
    · tau_d        : chord(현) 길이 게이트 [m] (기본 0.18)
    · min_pts_base : 최소 점수(점 개수) 기본값 (기본 8)
    · width_est    : 객체 가로폭 추정치 [m] (기본 0.30) → 동적 min_pts에 사용
    · visualize    : RViz 마커 퍼블리시 여부 (기본 True)
- Publish (Best Effort QoS):
    · /is_vehicle              (std_msgs/Bool)              : 검출 여부
    · /vehicle                 (std_msgs/Float32MultiArray) : [x, y, dist, angle_rad, count]
    · /vehicle_markers         (visualization_msgs/Marker)  : 클러스터 점/중심/ROI 박스 라인

설계 포인트:
- 극좌표에서 한 번 훑는 O(K) 벡터화. x,y는 사전계산 테이블과 곱셈 2회로 갱신 → 저지연.
- 연결 판단은 (1) 인접 인덱스 + (2) |Δr| 게이트 + (3) chord 게이트(d^2 < τ_d^2) 를 모두 만족할 때.
- 점수(=포인트 개수)로 물체 필터링, 거리 기반 동적 min_pts 적용으로 거리 변화에 강인.
- RViz2 마커: SPHERE_LIST(연녹색), 중심 SPHERE(노/주황), ROI 상자 LINE_STRIP(하늘색).

변경 요약:
- ✅ 물체 없을 때는 **/vehicle 퍼블리시 안 함**(이전엔 0 벡터 퍼블리시)
"""

from typing import Optional
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration


def make_qos_sensor() -> QoSProfile:
    """LiDAR/RViz 용 경량 Best Effort QoS"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


class VehicleDetector(Node):
    SCAN_TOPIC = '/scan'
    OUT_BOOL   = '/is_vehicle'
    OUT_VEC    = '/vehicle'
    OUT_MARKER = '/vehicle_markers'   # 하나의 토픽에 서로 다른 id/ns로 3종 마커 퍼블리시

    # Marker ID 예약
    MID_CLUSTER_POINTS = 0
    MID_CENTER_SPHERE  = 1
    MID_ROI_BOX        = 2

    def __init__(self):
        super().__init__('vehicle_detector')

        # ----- 파라미터 -----
        self.declare_parameter('roi_y_abs',    0.3)   # 좌우 ±0.4 m
        self.declare_parameter('roi_x_min',    0.1)   # 전방 0.0 m
        self.declare_parameter('roi_x_max',    1.2)   # 전방 2.0 m
        self.declare_parameter('tau_r',        0.90)  # 인접빔 거리차 [m]
        self.declare_parameter('tau_d',        0.90)  # chord 길이 [m]
        self.declare_parameter('min_pts_base', 1)     # 최소 점수
        self.declare_parameter('width_est',    0.001) # RC카 가로폭 추정
        self.declare_parameter('visualize',    True)

        self.roi_y_abs    = float(self.get_parameter('roi_y_abs').value)
        self.roi_x_min    = float(self.get_parameter('roi_x_min').value)
        self.roi_x_max    = float(self.get_parameter('roi_x_max').value)
        self.tau_r        = float(self.get_parameter('tau_r').value)
        self.tau_d        = float(self.get_parameter('tau_d').value)
        self.min_pts_base = int(self.get_parameter('min_pts_base').value)
        self.width_est    = float(self.get_parameter('width_est').value)
        self.visualize    = bool(self.get_parameter('visualize').value)

        qos = make_qos_sensor()
        self.sub = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.on_scan, qos)
        self.pub_flag   = self.create_publisher(Bool,               self.OUT_BOOL,   qos)
        self.pub_vector = self.create_publisher(Float32MultiArray,  self.OUT_VEC,    qos)
        self.pub_marker = self.create_publisher(Marker,             self.OUT_MARKER, qos)

        # ----- 캐시(초기 1회) -----
        self._N: Optional[int] = None
        self._angle_min: Optional[float] = None
        self._angle_inc: Optional[float] = None
        self._cos_t: Optional[np.ndarray] = None
        self._sin_t: Optional[np.ndarray] = None
        self._cos_d: Optional[float] = None
        self._frame_id = 'laser_frame'

        self.get_logger().info(
            f'VehicleDetector started (ROI=±{self.roi_y_abs:.2f} m, x∈[{self.roi_x_min:.1f}, {self.roi_x_max:.1f}] m).'
        )

    # ---------------- 내부 유틸 ----------------
    def _ensure_tables(self, msg: LaserScan):
        N = len(msg.ranges)
        if (self._N != N) or (self._angle_min != msg.angle_min) or (self._angle_inc != msg.angle_increment):
            self._N = N
            self._angle_min = msg.angle_min
            self._angle_inc = msg.angle_increment
            idx = np.arange(N, dtype=np.float32)
            theta = self._angle_min + idx * self._angle_inc
            self._cos_t = np.cos(theta).astype(np.float32)
            self._sin_t = np.sin(theta).astype(np.float32)
            self._cos_d = float(np.cos(self._angle_inc))  # 이웃빔 간 각도(상수)
            self.get_logger().info(f'Angle tables init: N={N}, inc={self._angle_inc:.6f} rad')
        if msg.header.frame_id:
            self._frame_id = msg.header.frame_id

    def _dynamic_min_pts(self, r_med: float) -> int:
        if r_med <= 0.0 or self._angle_inc is None:
            return self.min_pts_base
        # 예상 빔 수 ≈ (가로폭의 각도폭)/Δθ  (보수적으로 0.6배)
        alpha = 2.0 * math.atan(self.width_est / (2.0 * r_med))
        expect = alpha / max(self._angle_inc, 1e-6)
        return max(self.min_pts_base, int(expect * 0.6))

    # ✅ 변경: found=False일 때는 /vehicle 퍼블리시하지 않음
    def _publish_flag_vec(self, found: bool, x=None, y=None, dist=None, ang=None, count=None):
        # /is_vehicle
        msg_b = Bool(); msg_b.data = bool(found)
        self.pub_flag.publish(msg_b)

        # /vehicle — 오직 found=True일 때만 전송
        if not found:
            return

        vec = Float32MultiArray()
        vec.data = [float(x), float(y), float(dist), float(ang), float(count)]
        self.pub_vector.publish(vec)

    def _publish_cluster_markers(self, xs, ys, xc, yc, stamp):
        if not self.visualize:
            return
        # 1) 클러스터 점들
        m_pts = Marker()
        m_pts.header.frame_id = self._frame_id
        m_pts.header.stamp = stamp
        m_pts.ns = 'vehicle_cluster'
        m_pts.id = self.MID_CLUSTER_POINTS
        m_pts.type = Marker.SPHERE_LIST
        m_pts.action = Marker.ADD
        m_pts.scale.x = 0.04; m_pts.scale.y = 0.04; m_pts.scale.z = 0.04
        m_pts.color.r = 0.2;  m_pts.color.g = 1.0;  m_pts.color.b = 0.2;  m_pts.color.a = 0.9
        m_pts.lifetime = Duration(sec=0, nanosec=150_000_000)  # 0.15 s
        m_pts.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in zip(xs, ys)]
        self.pub_marker.publish(m_pts)

        # 2) 중심점
        m_c = Marker()
        m_c.header.frame_id = self._frame_id
        m_c.header.stamp = stamp
        m_c.ns = 'vehicle_center'
        m_c.id = self.MID_CENTER_SPHERE
        m_c.type = Marker.SPHERE
        m_c.action = Marker.ADD
        m_c.scale.x = 0.10; m_c.scale.y = 0.10; m_c.scale.z = 0.10
        m_c.color.r = 1.0;  m_c.color.g = 0.8;  m_c.color.b = 0.2;  m_c.color.a = 0.95
        m_c.lifetime = Duration(sec=0, nanosec=150_000_000)
        m_c.pose.position.x = float(xc)
        m_c.pose.position.y = float(yc)
        m_c.pose.position.z = 0.0
        self.pub_marker.publish(m_c)

    def _clear_cluster_markers(self, stamp):
        if not self.visualize:
            return
        for mid, ns in ((self.MID_CLUSTER_POINTS, 'vehicle_cluster'), (self.MID_CENTER_SPHERE, 'vehicle_center')):
            m = Marker()
            m.header.frame_id = self._frame_id
            m.header.stamp = stamp
            m.ns = ns
            m.id = mid
            m.action = Marker.DELETE
            self.pub_marker.publish(m)

    def _publish_roi_box(self, stamp):
        """ROI 사각형을 LINE_STRIP으로 표시 (x:[x_min,x_max], y:[-y_abs,+y_abs])"""
        if not self.visualize:
            return
        x0 = float(self.roi_x_min)
        x1 = float(self.roi_x_max)
        yA = float(self.roi_y_abs)
        y0 = -yA
        y1 = +yA

        corners = [
            Point(x=x0, y=y0, z=0.0),
            Point(x=x1, y=y0, z=0.0),
            Point(x=x1, y=y1, z=0.0),
            Point(x=x0, y=y1, z=0.0),
            Point(x=x0, y=y0, z=0.0),  # 닫기
        ]
        m = Marker()
        m.header.frame_id = self._frame_id
        m.header.stamp = stamp
        m.ns = 'vehicle_roi'
        m.id = self.MID_ROI_BOX
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02  # 선 굵기[m]
        m.color.r = 0.2; m.color.g = 0.8; m.color.b = 1.0; m.color.a = 0.9  # 하늘색
        m.lifetime = Duration(sec=0, nanosec=0)  # 0 → 지속표시
        m.points = corners
        self.pub_marker.publish(m)

    # ---------------- 메인 콜백 ----------------
    def on_scan(self, msg: LaserScan):
        self._ensure_tables(msg)

        # numpy 뷰 (복사 없이)
        r = np.asarray(msg.ranges, dtype=np.float32)

        # 유효/ROI 마스크
        valid = np.isfinite(r)
        if msg.range_min > 0.0:
            valid &= (r >= msg.range_min)
        if msg.range_max > 0.0:
            valid &= (r <= msg.range_max)

        # x=r cosθ, y=r sinθ  (벡터 곱 2회)
        x = r * self._cos_t
        y = r * self._sin_t

        roi = valid & (x >= self.roi_x_min) & (x <= self.roi_x_max) & (np.abs(y) <= self.roi_y_abs)
        idx = np.flatnonzero(roi)

        # ROI 박스는 항상 갱신 표시
        self._publish_roi_box(msg.header.stamp)

        # ▶ 물체 없음: /is_vehicle=False만 퍼블리시, /vehicle는 퍼블리시하지 않음
        if idx.size < 2:
            self._publish_flag_vec(False)
            self._clear_cluster_markers(msg.header.stamp)
            return

        # 인접성 + 게이트
        di = idx[1:] - idx[:-1]
        adjacent = (di == 1)

        r0 = r[idx[:-1]]
        r1 = r[idx[1:]]
        dr_ok = np.abs(r1 - r0) < self.tau_r

        # chord 게이트: d^2 = r0^2 + r1^2 - 2 r0 r1 cos(Δθ)
        d2 = r0 * r0 + r1 * r1 - 2.0 * r0 * r1 * self._cos_d
        chord_ok = d2 < (self.tau_d * self.tau_d)

        connected = adjacent & dr_ok & chord_ok

        # 끊긴 지점에서 분할
        boundaries = np.flatnonzero(~connected) + 1
        clusters = np.split(idx, boundaries)

        # 가장 점수 큰 클러스터 선택 + 동적 min_pts 적용
        best = None
        best_n = 0
        for c_idx in clusters:
            if c_idx.size == 0:
                continue
            r_med = float(np.median(r[c_idx]))
            min_pts = self._dynamic_min_pts(r_med)
            n = c_idx.size
            if n >= min_pts and n > best_n:
                best = c_idx
                best_n = n

        # ▶ 물체 없음: /is_vehicle=False만 퍼블리시, /vehicle는 퍼블리시하지 않음
        if best is None:
            self._publish_flag_vec(False)
            self._clear_cluster_markers(msg.header.stamp)
            return

        # 대표값(중심): median 사용
        xs = x[best]; ys = y[best]
        xc = float(np.median(xs))
        yc = float(np.median(ys))
        dist = math.hypot(xc, yc)
        ang  = math.atan2(yc, xc)   # 라디안

        # Publish — found=True일 때만 /vehicle 퍼블리시
        self._publish_flag_vec(True, xc, yc, dist, ang, best_n)
        self._publish_cluster_markers(xs, ys, xc, yc, msg.header.stamp)


def main():
    rclpy.init()
    node = VehicleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
