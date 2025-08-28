#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def quat_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    """쿼터니언 -> yaw(도). REP-103 ENU, +는 반시계(CCW)."""
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(yaw)


def wrap_deg(deg: float, mode: str) -> float:
    """'deg180' → [-180,180), 'deg360' → [0,360)."""
    if mode == 'deg360':
        d = deg % 360.0
        return d if d >= 0.0 else d + 360.0
    # default: deg180
    return (deg + 180.0) % 360.0 - 180.0


class ImuHeadingCalculator(Node):
    def __init__(self):
        super().__init__('imu_heading_calculator')

        # 기존 declare_parameter(...) 아래에 추가
        self.declare_parameter('auto_calib_sec', 0.0)     # 자동 오프셋 캘리브레이션 시간(초)
        self.declare_parameter('ema_alpha', 0.3)          # 0(느림)~1(빠름)
        self.declare_parameter('publish_unwrapped', True)

        self.auto_calib_sec = float(self.get_parameter('auto_calib_sec').value)
        self.alpha = float(self.get_parameter('ema_alpha').value)
        self.publish_unwrapped = bool(self.get_parameter('publish_unwrapped').value)

        # 추가 퍼블리셔
        from std_msgs.msg import Float64
        self.pub_unwrapped = self.create_publisher(Float64, '/heading_unwrapped', 10) if self.publish_unwrapped else None
        self.pub_rate = self.create_publisher(Float64, '/heading_rate', 10)

        # 내부 상태
        self._calib_until = self.get_clock().now() + rclpy.duration.Duration(seconds=self.auto_calib_sec)
        self._calib_sum = 0.0
        self._calib_cnt = 0
        self._prev_unwrapped = None
        self._filtered = None
        self._prev_time = None

    def _unwrap_shortest(self, prev_deg, curr_deg):
        """이전값 기준 가장 짧은 경로로 언랩."""
        delta = curr_deg - prev_deg
        while delta > 180.0:  delta -= 360.0
        while delta < -180.0: delta += 360.0
        return prev_deg + delta

    def imu_cb(self, msg: Imu):
        x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        if (x*x + y*y + z*z + w*w) < 1e-12:
            return

        yaw_deg = quat_to_yaw_deg(x, y, z, w)  # 기존 함수
        raw_heading = wrap_deg(yaw_deg + self.offset, self.mode)

        # 1) 자동 오프셋 캘리브레이션
        now = self.get_clock().now()
        if self.auto_calib_sec > 0 and now < self._calib_until:
            self._calib_sum += raw_heading
            self._calib_cnt += 1
            return
        elif self.auto_calib_sec > 0 and self._calib_cnt > 0 and abs(self.offset) < 1e-9:
            avg = self._calib_sum / self._calib_cnt
            # 평균을 0°로 맞추도록 offset을 보정
            self.offset -= avg
            self.get_logger().info(f'Auto-calibrated heading_offset_deg: {self.offset:.2f}')
            self._calib_cnt = 0

        # 2) 언랩
        if self._prev_unwrapped is None:
            unwrapped = raw_heading
        else:
            unwrapped = self._unwrap_shortest(self._prev_unwrapped, raw_heading)
        self._prev_unwrapped = unwrapped

        # 3) EMA 필터
        if self._filtered is None:
            self._filtered = unwrapped
        else:
            self._filtered = (1.0 - self.alpha) * self._filtered + self.alpha * unwrapped

        # 4) yaw rate (deg/s)
        t = now
        if self._prev_time is None:
            rate_deg_s = 0.0
        else:
            dt = (t - self._prev_time).nanoseconds * 1e-9
            if dt > 1e-6:
                rate_deg_s = (unwrapped - self._prev_unwrapped) / dt
            else:
                rate_deg_s = 0.0
        self._prev_time = t

        # 퍼블리시(원래 /heading은 랩핑된 값, 제어에는 보통 filtered/unwrapped 사용 권장)
        out = Float64(); out.data = wrap_deg(self._filtered, self.mode)
        self.pub.publish(out)

        if self.pub_unwrapped:
            out2 = Float64(); out2.data = self._filtered  # 언랩/필터된 연속 값
            self.pub_unwrapped.publish(out2)

        out3 = Float64(); out3.data = rate_deg_s
        self.pub_rate.publish(out3)


def main():
    rclpy.init()
    node = ImuHeadingCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()