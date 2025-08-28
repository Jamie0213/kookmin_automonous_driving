#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge
import numpy as np

class OrangeDetector(Node):
    def __init__(self):
        super().__init__('orange_detector')

        # ---- Parameters ----
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('roi_y_fraction', 0.5)          # 하단 절반부터 검사
        self.declare_parameter('orange_ratio_threshold', 0.01) # 면적비 임계값
        self.declare_parameter('show_windows', True)            # imshow 사용 여부
        self.declare_parameter('display_scale', 1.0)            # 디버그 창 리사이즈 배율

        cam_topic  = self.get_parameter('camera_topic').value
        self.roi_f = float(self.get_parameter('roi_y_fraction').value)
        self.thres = float(self.get_parameter('orange_ratio_threshold').value)
        self.show  = bool(self.get_parameter('show_windows').value)
        self.scale = float(self.get_parameter('display_scale').value)

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.bridge = CvBridge()
        self.is_orange_pub = self.create_publisher(Bool, '/is_orange', 10)
        self.create_subscription(Image, cam_topic, self.on_image, qos)

        # HSV 범위(주황) — 현장에 맞게 조정
        self.hsv_low  = np.array([0,  111, 139], dtype=np.uint8)
        self.hsv_high = np.array([25, 255, 255], dtype=np.uint8)

        # 모폴로지 커널
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))

        self.get_logger().info(
            f"Orange detector (ROI>= {self.roi_f:.2f}h) listening on {cam_topic} -> /is_orange"
        )

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        h, w = frame.shape[:2]
        y0 = max(0, min(h - 1, int(self.roi_f * h)))  # ROI 시작 y
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ----- ROI에서만 색 검출 -----
        hsv_roi  = hsv[y0:, :]
        mask_roi = cv2.inRange(hsv_roi, self.hsv_low, self.hsv_high)
        mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_OPEN,  self.kernel, iterations=1)
        mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_CLOSE, self.kernel, iterations=1)

        orange_pixels = int(cv2.countNonZero(mask_roi))
        total_pixels  = int(mask_roi.size)
        orange_ratio  = (orange_pixels / total_pixels) if total_pixels > 0 else 0.0

        # 판정 & 퍼블리시 (매 프레임)
        msg_bool = Bool()
        msg_bool.data = orange_ratio >= self.thres
        self.is_orange_pub.publish(msg_bool)

        # ----- 전체 크기 마스크 복원 및 디버그 오버레이 -----
        mask_full = np.zeros((h, w), dtype=np.uint8)
        mask_full[y0:, :] = mask_roi

        debug = frame.copy()
        # ROI 박스
        cv2.rectangle(debug, (0, y0), (w-1, h-1), (255, 255, 0), 2)
        # 마스크 픽셀을 초록색으로 덮기
        overlay_roi = debug[y0:, :].copy()
        overlay_roi[mask_roi > 0] = [0, 255, 0]
        debug[y0:, :] = overlay_roi

        # 텍스트(픽셀 수/비율/판정)
        txt = (f"orange_pixels={orange_pixels} / total={total_pixels}  "
               f"ratio={orange_ratio:.4f}  thr={self.thres:.4f}  is_orange={msg_bool.data}")
        cv2.putText(debug, txt, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(debug, txt, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2, cv2.LINE_AA)

        # ----- 로그: 매 프레임 찍기 -----
        # self.get_logger().info(
        #     f"[ROI y>={y0}] orange_pixels={orange_pixels}, total={total_pixels}, "
        #     f"ratio={orange_ratio:.4f}, thr={self.thres:.4f}, is_orange={msg_bool.data}"
        # )

        # ----- imshow 창 표시 -----
        if self.show:
            vis_debug = debug
            vis_mask  = mask_full
            if self.scale != 1.0:
                vis_debug = cv2.resize(debug, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_LINEAR)
                vis_mask  = cv2.resize(mask_full, None, fx=self.scale, fy=self.scale, interpolation=cv2.NEAREST)

            #cv2.imshow("Orange Debug (BGR overlay)", vis_debug)
            #cv2.imshow("Orange Mask (mono8)", vis_mask)
            # 키 입력 처리 (창 이벤트 갱신 필수)
            key = cv2.waitKey(1) & 0xFF
            # 필요시 'q'로 창 닫고 노드 종료 원하면 아래 주석 해제
            # if key == ord('q'):
            #     self.get_logger().info("Quit requested by user keypress.")
            #     rclpy.shutdown()

def main():
    rclpy.init()
    node = OrangeDetector()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
