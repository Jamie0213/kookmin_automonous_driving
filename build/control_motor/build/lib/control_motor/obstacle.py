#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# ROS 2 – ObstacleClassifier (BEST-EFFORT QoS 적용)
# ------------------------------------------------------------
# · LiDAR 스캔을 클러스터링 → vehicle / rubbercone 판별
# · 결과를 /obstacle(String) 으로 퍼블리시
# ------------------------------------------------------------

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos  import qos_profile_sensor_data          # ★ BEST-EFFORT QoS
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from typing import List

# ───────── 파라미터 ─────────
Y_MAX           = 8.0       # ROI 전방 상한 [m]
FWD_MIN         = 0.75      # 전방 최소 거리 [m]
CONE_MIN_PTS    = 10         # 라바콘 최소 점 개수
VEHICLE_MIN_PTS = 35        # 차량 최소 점 개수
CLUSTER_GAP     = 0.05       # 인접 포인트 최대 거리 [m]
# ───────────────────────────


class ObstacleClassifier(Node):
    def __init__(self):
        super().__init__("obstacle_classifier")

        # 퍼블리셔
        self.pub_obstacle = self.create_publisher(String, "/obstacle", 10)

        # ────────── BEST-EFFORT QoS 구독 ──────────
        self.create_subscription(
            LaserScan, "/scan", self.lidar_cb,
            qos_profile_sensor_data)                      # ★
        # ------------------------------------------

        self.last_result = None        # 중복 퍼블리시 방지
        self.get_logger().info("obstacle_classifier ▶ READY")

    # ─────────────────────────────────────────────
    def cluster_points(self, x, y) -> List[np.ndarray]:
        """CLUSTER_GAP 이내 이웃을 묶어 인덱스 리스트 반환"""
        pts, used, clusters = np.vstack([x, y]).T, np.zeros(len(x), bool), []
        for i in range(len(pts)):
            if used[i]:
                continue
            cluster, used[i], q = [i], True, [i]
            while q:
                idx = q.pop()
                dist = np.linalg.norm(pts - pts[idx], axis=1)
                neigh = np.where((dist < CLUSTER_GAP) & (~used))[0]
                for n in neigh:
                    used[n] = True
                    cluster.append(n)
                    q.append(n)
            clusters.append(cluster)
        return clusters

    # ─────────────────────────────────────────────
    def lidar_cb(self, scan: LaserScan):
        """LiDAR 데이터 처리"""
        r = np.asarray(scan.ranges)
        ang = scan.angle_min + np.arange(len(r)) * scan.angle_increment

        roi = (r > FWD_MIN) & (r < Y_MAX) & (np.abs(ang) < np.pi/2)
        if roi.sum() == 0:
            return

        xf = r[roi] * np.cos(ang[roi])   # 전방
        yl = -r[roi] * np.sin(ang[roi])  # 좌우 (왼− / 오+)

        clusters = self.cluster_points(xf, yl)

        cone_cnt = sum(1 for c in clusters if CONE_MIN_PTS    <= len(c) < VEHICLE_MIN_PTS)
        veh_cnt  = sum(1 for c in clusters if len(c) >= VEHICLE_MIN_PTS)

        result = None
        if veh_cnt > 0:
            result = "vehicle"
        elif cone_cnt >= 2:
            result = "rubbercone"

        if result and result != self.last_result:
            msg = String(); msg.data = result
            self.pub_obstacle.publish(msg)
            self.last_result = result
            self.get_logger().info(f"Detected: {result}")


# ------------------------------------------------------------
def main():
    rclpy.init()
    node = ObstacleClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
