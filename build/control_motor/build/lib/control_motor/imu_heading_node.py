# ===============================================
# File: imu_heading_node.py
# Desc: /imu (sensor_msgs/Imu) → /heading(std_msgs/Float32, deg)
#       + /diagnostics(diagnostic_msgs/DiagnosticArray) with 'yaw (deg)'
# Notes:
#  - Computes yaw (heading) from quaternion. Normalizes to [-180, 180] deg.
#  - Optional offset parameter yaw_offset_deg for calibration (e.g., to align car's forward to 0°).
#  - Simple EMA smoothing via param heading_ema_alpha (0~1).
#  - QoS BEST_EFFORT for sensor streams.
# ===============================================

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def qos_sensor_best_effort(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def quat_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    # yaw (Z) from quaternion
    # Ref: yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)  # rad
    return math.degrees(yaw)


def wrap_deg180(a: float) -> float:
    # Wrap to [-180, 180]
    a = (a + 180.0) % 360.0 - 180.0
    return a


class ImuHeadingNode(Node):
    def __init__(self):
        super().__init__('imu_heading')

        # ---- Parameters ----
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('heading_ema_alpha', 0.1)  # 0(no smooth) ~ 1(strong smooth)
        self.declare_parameter('publish_diagnostics', True)

        self._yaw_offset = float(self.get_parameter('yaw_offset_deg').value)
        self._alpha = float(self.get_parameter('heading_ema_alpha').value)
        self._pub_diag = bool(self.get_parameter('publish_diagnostics').value)

        # ---- State ----
        self._heading_deg_ema: Optional[float] = None

        # ---- I/O ----
        self.sub_imu = self.create_subscription(Imu, '/imu', self.cb_imu, qos_sensor_best_effort(20))
        self.pub_heading = self.create_publisher(Float32, '/heading', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10) if self._pub_diag else None

        self.get_logger().info('imu_heading node started. Subscribing /imu, publishing /heading (deg).')

    def cb_imu(self, msg: Imu):
        q = msg.orientation
        yaw_deg = quat_to_yaw_deg(q.x, q.y, q.z, q.w)
        yaw_deg = wrap_deg180(yaw_deg + self._yaw_offset)

        if self._heading_deg_ema is None:
            self._heading_deg_ema = yaw_deg
        else:
            a = max(0.0, min(1.0, self._alpha))
            self._heading_deg_ema = (1.0 - a) * yaw_deg + a * self._heading_deg_ema

        # Publish heading (deg)
        out = Float32()
        out.data = float(self._heading_deg_ema)
        self.pub_heading.publish(out)

        # Optional diagnostics
        if self.pub_diag is not None:
            da = DiagnosticArray()
            st = DiagnosticStatus(level=DiagnosticStatus.OK, name='imu_heading', message='ok')
            st.values = [
                KeyValue(key='yaw (deg)', value=f'{self._heading_deg_ema:.3f}'),
                KeyValue(key='yaw_raw (deg)', value=f'{yaw_deg:.3f}'),
                KeyValue(key='offset (deg)', value=f'{self._yaw_offset:.3f}')
            ]
            da.status.append(st)
            da.header.stamp = self.get_clock().now().to_msg()
            self.pub_diag.publish(da)


def main():
    rclpy.init()
    node = ImuHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()