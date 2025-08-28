#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 – VehicleAvoidance 노드 (BEST-EFFORT QoS 적용)
------------------------------------------------------------
· /obstacle(String) · /lane_index(Int32) · /scan(LaserScan)  구독
· 전방 ROI 내 ‘vehicle’ 감지 시 좌/우 회피 후 복귀
· /vehicle(Float32MultiArray)  →  [angle, speed, flag] 퍼블리시
"""

import math, time, rclpy
from rclpy.node import Node
from rclpy.qos  import qos_profile_sensor_data          # ★ BEST-EFFORT QoS
from std_msgs.msg import String, Int32, Float32MultiArray
from sensor_msgs.msg import LaserScan

# ─────── 파라미터 ───────
STEER_LIMIT   = 100.0
STEER_VAL     = 40.0
SPEED_CONST   = 5.0
BLOCK_TIME    = 3.0

ROI_DIST      = 5.5
ROI_ANG_DEG   = 10.0          # ±5°
# ────────────────────────


class VehicleAvoidance(Node):
    def __init__(self):
        super().__init__("vehicle_avoidance_node")

        # 퍼블리셔
        self.pub_motor = self.create_publisher(
            Float32MultiArray, "/vehicle", 10)

        # ────────────── 구독자 ──────────────
        self.create_subscription(
            String, "/obstacle", self.obstacle_cb, 10)    # 기본(신뢰성 必) 유지
        self.create_subscription(
            Int32, "/lane_index", self.lane_cb, 10)
        self.create_subscription(                       # ★ LiDAR: BEST-EFFORT
            LaserScan, "/scan", self.scan_cb,
            qos_profile_sensor_data)
        # ------------------------------------

        self.lane_idx      = None
        self.obstacle_type = None
        self.is_handling   = False
        self.reset_timer   = None

        self.get_logger().info("▶ vehicle_avoidance_node READY")

    # ───────── 콜백 ─────────
    def lane_cb(self, msg: Int32):
        self.lane_idx = msg.data

    def obstacle_cb(self, msg: String):
        self.obstacle_type = msg.data

    def scan_cb(self, scan: LaserScan):
        if self.is_handling or self.lane_idx is None:
            return
        if self.obstacle_type != "vehicle":
            return

        roi_ang = math.radians(ROI_ANG_DEG)
        i_min = max(int((-roi_ang/2 - scan.angle_min) / scan.angle_increment), 0)
        i_max = min(int(( roi_ang/2 - scan.angle_min) / scan.angle_increment),
                    len(scan.ranges) - 1)

        if any(scan.range_min < d < ROI_DIST
               for d in scan.ranges[i_min:i_max + 1]):
            self.try_avoidance()

    # ─────── 회피 로직 ───────
    def try_avoidance(self):
        if self.is_handling or self.lane_idx is None:
            return
        self.is_handling = True

        if self.lane_idx == 1:       # 1차선 → 우측
            steer, duration =  STEER_VAL, 1.2
        else:                        # 2차선 → 좌측
            steer, duration = -STEER_VAL, 1.4

        self.get_logger().info(f"vehicle 감지 → 조향 {steer:+.1f}°, {duration}s")

        # ① 회피
        self.steer_for_duration(steer,  duration)
        # ② 복귀
        self.steer_for_duration(-steer, duration)
        # ③ 재감지 차단
        self.reset_timer = self.create_timer(BLOCK_TIME, self.reset_handling)

    def reset_handling(self):
        self.is_handling = False
        if self.reset_timer:
            self.reset_timer.cancel()
            self.reset_timer = None

    # ─────── 모터 퍼블리시 ───────
    def steer_for_duration(self, angle: float, duration: float):
        angle = max(min(angle, STEER_LIMIT), -STEER_LIMIT)
        cmd = Float32MultiArray()
        cmd.data = [angle, SPEED_CONST, 0.0] # if you want use, flag = 1.0

        start = time.time()
        while (time.time() - start) < duration and rclpy.ok():
            self.pub_motor.publish(cmd)
            time.sleep(0.1)          # 10 Hz

        cmd.data = [0.0, SPEED_CONST, 0.0]
        self.pub_motor.publish(cmd)


# ──────────────────────────
def main():
    rclpy.init()
    node = VehicleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
