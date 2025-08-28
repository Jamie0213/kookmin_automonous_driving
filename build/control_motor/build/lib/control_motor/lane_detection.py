#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 – LaneDetection 노드 (HSV 트랙바 + 디버그 버전, Float32MultiArray 메시지)
------------------------------------------------------------
· /usb_cam/image_raw  → 카메라 프레임 입력
· 차선(노란·흰 선) 검출 + 슬라이딩-윈도우 중앙 오차 계산
· 모터 명령 /lane_detection (Float32MultiArray) 퍼블리시
· 차선 번호 /lane_index    (Int32)              퍼블리시

디버그 창
 01_Orig          : 원본 프레임
 02_YellowMask    : 노란선 HSV 마스크
 03_WhiteMask     : 흰선 HSV 마스크
 04_ORMask        : 노란·흰 OR 마스크
 05_Warped        : 버드아이(Bird-Eye) 컬러
 06_WarpedMask    : 버드아이 노란선 마스크
 07_Slide         : 슬라이딩-윈도우 결과
 HSV_Tuner        : HSV 범위 트랙바(12개)
------------------------------------------------------------
"""

import os, sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg    import Int32, Float32MultiArray      # ← 메시지 타입 변경
from cv_bridge import CvBridge

# 로컬 모듈(slidewindow.py) 경로 추가
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from slidingwindow import SlideWindow

# ───── 고정 파라미터 ─────
CENTER_X      = 320
STEER_LIMIT   = 100.0
DEFAULT_SPEED = 5.0
CURVE_SPEED   = 5.0
DEFAULT_GAIN  = 0.01
CURVE_GAIN    = 0.25
LANE_MIN_PIX  = 500

# HSV 초기값 (트랙바 기본값)
INIT_LOWER_Y = (24, 73, 109)
INIT_UPPER_Y = (170, 255, 255)
INIT_LOWER_W = (46,   0, 129)
INIT_UPPER_W = (180, 17, 255)

DEBUG_WINDOWS = True     # 디버그 창 ON/OFF
WIN_SCALE     = 1.5      # 창 크기 배율 (1.0 = 원본)
IMG_W, IMG_H  = 640, 480 # 디버그용 표준 크기
# ─────────────────────────


def _nothing(x: int) -> None:
    """트랙바 콜백(사용 안 함)"""
    pass


class LaneDetectionROS(Node):
    """Lane Detection & Motor Control (with debug visualizers + HSV tuner)"""

    def __init__(self):
        super().__init__("lane_detection_node")
        self.bridge      = CvBridge()
        self.slidewindow = SlideWindow()

        # ROS I/O ---------------------------------------------------------
        self.sub_image = self.create_subscription(
            Image, "/image_raw", self.image_callback, 10)
        self.pub_motor = self.create_publisher(
            Float32MultiArray, "/lane_detection", 10)        # ← 타입 변경
        self.pub_lane  = self.create_publisher(
            Int32,              "/lane_index",     10)

        # 내부 상태 --------------------------------------------------------
        self.cv_image: np.ndarray | None = None
        self.lane_msg  = Int32()
        self.motor_msg = Float32MultiArray()                 # ← 타입 변경

        # 디버그 창/트랙바 초기화 여부
        self.windows_initialized = False
        self.trackbar_initialized = False

        # 20 Hz 처리 루프
        self.create_timer(0.05, self.process_frame)
        self.get_logger().info("lane_detection_node ▶ READY")

    # ──────────────────────────────────────────
    def setup_trackbars(self):
        """HSV_Tuner 창과 트랙바 생성(한 번만 호출)"""
        if self.trackbar_initialized:
            return

        cv2.namedWindow("HSV_Tuner", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("HSV_Tuner", 400, 350)

        # 노란선(YL_) 트랙바
        for name, val, maxv in [
            ("YL_H_LOW",  INIT_LOWER_Y[0], 180),
            ("YL_H_HIGH", INIT_UPPER_Y[0], 180),
            ("YL_S_LOW",  INIT_LOWER_Y[1], 255),
            ("YL_S_HIGH", INIT_UPPER_Y[1], 255),
            ("YL_V_LOW",  INIT_LOWER_Y[2], 255),
            ("YL_V_HIGH", INIT_UPPER_Y[2], 255),
        ]:
            cv2.createTrackbar(name, "HSV_Tuner", val, maxv, _nothing)

        # 흰선(WT_) 트랙바
        for name, val, maxv in [
            ("WT_H_LOW",  INIT_LOWER_W[0], 180),
            ("WT_H_HIGH", INIT_UPPER_W[0], 180),
            ("WT_S_LOW",  INIT_LOWER_W[1], 255),
            ("WT_S_HIGH", INIT_UPPER_W[1], 255),
            ("WT_V_LOW",  INIT_LOWER_W[2], 255),
            ("WT_V_HIGH", INIT_UPPER_W[2], 255),
        ]:
            cv2.createTrackbar(name, "HSV_Tuner", val, maxv, _nothing)

        self.trackbar_initialized = True

    # ──────────────────────────────────────────
    def read_hsv_from_trackbar(self):
        """트랙바 값 → HSV 하·상한 ndarray 반환"""
        yl_l = np.array([
            cv2.getTrackbarPos("YL_H_LOW",  "HSV_Tuner"),
            cv2.getTrackbarPos("YL_S_LOW",  "HSV_Tuner"),
            cv2.getTrackbarPos("YL_V_LOW",  "HSV_Tuner"),
        ])
        yl_u = np.array([
            cv2.getTrackbarPos("YL_H_HIGH", "HSV_Tuner"),
            cv2.getTrackbarPos("YL_S_HIGH", "HSV_Tuner"),
            cv2.getTrackbarPos("YL_V_HIGH", "HSV_Tuner"),
        ])
        wt_l = np.array([
            cv2.getTrackbarPos("WT_H_LOW",  "HSV_Tuner"),
            cv2.getTrackbarPos("WT_S_LOW",  "HSV_Tuner"),
            cv2.getTrackbarPos("WT_V_LOW",  "HSV_Tuner"),
        ])
        wt_u = np.array([
            cv2.getTrackbarPos("WT_H_HIGH", "HSV_Tuner"),
            cv2.getTrackbarPos("WT_S_HIGH", "HSV_Tuner"),
            cv2.getTrackbarPos("WT_V_HIGH", "HSV_Tuner"),
        ])
        return yl_l, yl_u, wt_l, wt_u

    # ──────────────────────────────────────────
    def image_callback(self, msg: Image):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")

    # ──────────────────────────────────────────
    def process_frame(self):
        if self.cv_image is None:
            return

        # 디버그/트랙바 초기화(최초 1회)
        if DEBUG_WINDOWS and not self.trackbar_initialized:
            self.setup_trackbars()

        frame = cv2.resize(self.cv_image, (IMG_W, IMG_H))

        # 1. HSV 마스크(노란·흰 선) ------------------------------
        # - 트랙바로부터 현재 HSV 값 읽기
        lower_y, upper_y, lower_w, upper_w = self.read_hsv_from_trackbar()

        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_y  = cv2.inRange(img_hsv, lower_y, upper_y)
        mask_w  = cv2.inRange(img_hsv, lower_w, upper_w)
        mask    = cv2.bitwise_or(mask_y, mask_w)
        filt    = cv2.bitwise_and(frame, frame, mask=mask)

        # 2. 버드아이 변환 ----------------------------------------
        h, w = frame.shape[:2]
        src_pts = np.float32([[130, 415], [305, 273],
                              [455, 273], [610, 415]])
        mid = w // 2
        dst_pts = np.float32([[160, h-60], [160, 0],
                              [480, 0],   [480, h-60]])
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        warped     = cv2.warpPerspective(filt,    M, (IMG_W, IMG_H))
        warped_y   = cv2.warpPerspective(mask_y, M, (IMG_W, IMG_H))

        # 3. 슬라이딩-윈도우 --------------------------------------
        gray   = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        binary = (gray > 20).astype(np.uint8)
        out_img, x_loc, _ = self.slidewindow.slidewindow(binary)

        # 4. 차선 인덱스 ------------------------------------------
        roi = warped_y[400:, :]
        left_pix  = np.sum(roi[:, :IMG_W//2]  > 0)
        right_pix = np.sum(roi[:, IMG_W//2:] > 0)
        if right_pix > left_pix and right_pix > LANE_MIN_PIX:
            lane_idx = 1
        elif left_pix > right_pix and left_pix > LANE_MIN_PIX:
            lane_idx = 2
        else:
            lane_idx = self.lane_msg.data
        self.lane_msg.data = lane_idx
        self.pub_lane.publish(self.lane_msg)

        # 5. 조향·속도 --------------------------------------------
        err_px = x_loc - CENTER_X
        gain   = DEFAULT_GAIN if abs(err_px) < 100 else CURVE_GAIN
        angle  = float(np.clip(err_px * gain, -STEER_LIMIT, STEER_LIMIT))
        speed  = DEFAULT_SPEED if abs(err_px) < 100 else CURVE_SPEED

        # ← Float32MultiArray: [angle, speed, flag]
        self.motor_msg.data = [angle, float(speed), 1.0]
        self.pub_motor.publish(self.motor_msg)

        # 6. 디버그 시각화 ----------------------------------------
        if DEBUG_WINDOWS:
            titles = ["01_Orig", "02_YellowMask", "03_WhiteMask",
                      "04_ORMask", "05_Warped", "06_WarpedMask", "07_Slide"]

            if not self.windows_initialized:
                for t in titles:
                    cv2.namedWindow(t, cv2.WINDOW_NORMAL)
                    if WIN_SCALE != 1.0:
                        cv2.resizeWindow(t,
                                         int(IMG_W * WIN_SCALE),
                                         int(IMG_H * WIN_SCALE))
                self.windows_initialized = True
    
            cv2.imshow("01_Orig",        frame)
            cv2.imshow("02_YellowMask",  cv2.cvtColor(mask_y,  cv2.COLOR_GRAY2BGR))
            cv2.imshow("03_WhiteMask",   cv2.cvtColor(mask_w,  cv2.COLOR_GRAY2BGR))
            # 필요 시 다른 창 활성화
            cv2.imshow("04_ORMask",      cv2.cvtColor(mask,    cv2.COLOR_GRAY2BGR))
            cv2.imshow("05_Warped",      warped)
            cv2.imshow("06_WarpedMask",  cv2.cvtColor(warped_y, cv2.COLOR_GRAY2BGR))
            cv2.imshow("07_Slide",       cv2.resize(out_img, (IMG_W, IMG_H)))
            cv2.waitKey(1)


# ──────────────────────────────────────────
def main():
    rclpy.init()
    node = LaneDetectionROS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
