#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
multi_cmd_relay  - 최종 출력: XycarMotor
------------------------------------------------------------
· 여러 모드(Float32MultiArray) 중 우선순위가 가장 높은 것을 골라
  최종 토픽 /xycar_motor(xycar_msgs/XycarMotor)로 릴레이합니다.

입력 Float32MultiArray.data 인덱스
  [0] angle (float32)
  [1] speed (float32)
  [2] flag  (float32; 0.0=비활성, 0.0 이외=활성)

출력 XycarMotor
  header, angle(float32), speed(float32), flag(bool)
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from functools import partial
from typing import Dict

from std_msgs.msg import Float32MultiArray
from xycar_msgs.msg import XycarMotor   # ← 추가

# ───────── 우선순위 목록 ─────────
TOPIC_ORDER = [
    ("traffic_light", "/traffic_light"),   # 1
    ("rubbercone",    "/rubbercone"),      # 2
    ("vehicle",       "/avoidance_motor"),# 3
    ("lane_detection","/lane_detection"),  # 4 (기본)
]
# ────────────────────────────────

# ───────── 워밍업 설정 ─────────
WARMUP_DURATION = 0.0   # s
WARMUP_SPEED    = 10.0
WARMUP_ANGLE    = 0.0
# ──────────────────────────────

def _get_field(msg: Float32MultiArray, idx: int, default: float = 0.0) -> float:
    """msg.data[idx] 안전 추출 (길이 부족하면 default 반환)"""
    return float(msg.data[idx]) if len(msg.data) > idx else default

class MultiCmdRelay(Node):
    def __init__(self):
        super().__init__("kmu_motor_node")

        # 최종 모터 퍼블리셔 (XycarMotor로 변경)
        self.pub_motor = self.create_publisher(XycarMotor, "/xycar_motor", 10)

        # 각 모드별 최신 메시지 (입력은 그대로 Float32MultiArray)
        empty = Float32MultiArray(data=[0.0, 0.0, 0.0])
        self.latest: Dict[str, Float32MultiArray] = {
            name: empty for name, _ in TOPIC_ORDER
        }

        # 모드 전환·워밍업 상태
        self.current_src = ""
        self.prev_src    = ""
        self.lane_warmup_until: Time = self.get_clock().now()

        # 우선순위별 구독자 등록
        for name, topic in TOPIC_ORDER:
            self.create_subscription(
                Float32MultiArray,
                topic,
                partial(self._cb, source=name),
                10
            )

        self.get_logger().info("multi_cmd_relay ▶ READY (out: XycarMotor)")

    # ────────────────────────────────────
    def _cb(self, msg: Float32MultiArray, source: str):
        """모든 입력 토픽의 공통 콜백"""
        self.latest[source] = msg
        self._publish_highest_priority()

    # ────────────────────────────────────
    def _publish_highest_priority(self):
        # 1) flag!=0 인 토픽 중 가장 높은 우선순위 찾기
        active_src = next(
            (name for name, _ in TOPIC_ORDER
             if _get_field(self.latest[name], 2) != 0.0),
            "lane_detection"  # 아무 flag도 없으면 기본
        )

        # 2) 모드가 바뀌면 워밍업 여부 계산
        if active_src != self.current_src:
            self.prev_src, self.current_src = self.current_src, active_src

            if (active_src == "lane_detection"
                    and self.prev_src in ("rubbercone", "traffic_light")):
                self.lane_warmup_until = (
                    self.get_clock().now() + Duration(seconds=WARMUP_DURATION)
                )
            else:
                self.lane_warmup_until = self.get_clock().now()

        # 3) XycarMotor 메시지 구성
        now = self.get_clock().now()
        out = XycarMotor()
        out.header.stamp = now.to_msg()
        out.header.frame_id = ""  # 필요 시 'base_link' 등으로 지정

        if (self.current_src == "lane_detection" and now < self.lane_warmup_until):
            # 워밍업 구간: 고정 speed/angle, flag=False
            out.angle = float(WARMUP_ANGLE)
            out.speed = float(WARMUP_SPEED)
            out.flag  = False
        else:
            src_cmd = self.latest[self.current_src]
            out.angle = _get_field(src_cmd, 0)
            out.speed = _get_field(src_cmd, 1)
            out.flag  = True

        # 4) 최종 퍼블리시
        self.pub_motor.publish(out)

# ────────────────────────────────────
def main():
    rclpy.init()
    node = MultiCmdRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
