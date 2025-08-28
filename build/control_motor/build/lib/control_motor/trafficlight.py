#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
traffic_light_detector (start-latch + after-start always flag=0)
"""

import cv2, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg    import Float32MultiArray, Bool
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# ---- 파라미터 ----
ROI_Y_MAX     = 220
PIXEL_THRESH  = 30
DRIVE_SPEED   = 50.0
DEBUG         = False
GREEN_LATCH_N = 2   # GREEN N프레임 연속 관찰 시 출발 확정

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__("trafficlight")
        self.bridge = CvBridge()

        self.create_subscription(Image, "/image_raw", self.image_cb, 10)
        self.pub_motor = self.create_publisher(Float32MultiArray, "/traffic_light", 10)

        # /race_started는 라칭 QoS(TRANSIENT_LOCAL)로 퍼블리시 → 늦게 구독해도 마지막 True 수신
        start_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.pub_start = self.create_publisher(Bool, "/race_started", start_qos)

        # HSV (필요시 현장에서 튜닝)
        self.lower_red1, self.upper_red1 = np.uint8([1,141,228]), np.uint8([180,255,255])
        self.lower_red2, self.upper_red2 = np.uint8([1,141,228]), np.uint8([180,255,255])
        self.lower_yellow, self.upper_yellow = np.uint8([0,20,255]), np.uint8([91,255,255])
        self.lower_green,  self.upper_green  = np.uint8([61,155,111]), np.uint8([132,255,255])

        if DEBUG:
            cv2.namedWindow("ROI", cv2.WINDOW_NORMAL)
            cv2.namedWindow("MASK-RED", cv2.WINDOW_NORMAL)
            cv2.namedWindow("MASK-YELLOW", cv2.WINDOW_NORMAL)
            cv2.namedWindow("MASK-GREEN", cv2.WINDOW_NORMAL)

        self._race_started = False
        self._green_streak = 0
        self._start_published = False

        self.get_logger().info("traffic_light_detector ▶ READY (GREEN→/race_started latch, 이후 flag=0)")

    def image_cb(self, msg: Image):
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        roi = img_bgr[:ROI_Y_MAX, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mask_r = cv2.bitwise_or(
            cv2.inRange(hsv, self.lower_red1, self.upper_red1),
            cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        )
        mask_y = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        mask_g = cv2.inRange(hsv, self.lower_green,  self.upper_green)

        cnt_r = cv2.countNonZero(mask_r)
        cnt_g = cv2.countNonZero(mask_g)

        cmd = Float32MultiArray()

        if not self._race_started:
            # GREEN 연속 감지로 출발 확정
            self._green_streak = self._green_streak + 1 if cnt_g > PIXEL_THRESH else 0
            if self._green_streak >= GREEN_LATCH_N:
                self._race_started = True
                if not self._start_published:
                    self.pub_start.publish(Bool(data=True))
                    self._start_published = True
                    self.get_logger().info("/race_started = True (latched)")
                cmd.data = [0.0, DRIVE_SPEED, 0.0]   # 이 순간부터는 flag=0
            else:
                # 빨간불이면 정지(우선권 보장), 그 외엔 주행 유지
                if cnt_r > PIXEL_THRESH:
                    cmd.data = [0.0, 0.0, 1.0]
                else:
                    cmd.data = [0.0, DRIVE_SPEED, 0.0]
        else:
            # 출발 이후에는 영구적으로 flag=0
            cmd.data = [0.0, DRIVE_SPEED, 0.0]

        self.pub_motor.publish(cmd)

        if DEBUG:
            cv2.imshow("ROI", roi)
            cv2.imshow("MASK-RED", mask_r)
            cv2.imshow("MASK-YELLOW", mask_y)
            cv2.imshow("MASK-GREEN", mask_g)
            cv2.waitKey(1)

def main():
    rclpy.init()
    node = TrafficLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if DEBUG:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
