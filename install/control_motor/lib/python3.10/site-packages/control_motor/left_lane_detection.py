#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 – LaneDetection (new_utils + new_slidingwindow, 저부하 디버그)
"""

import os, sys, cv2, numpy as np, rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32MultiArray
from cv_bridge import CvBridge

# 로컬 모듈
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from left_slidingwindow import SlideWindow
from left_utils import *   # roi_for_lane, process_image, warper 등

# ───── 파라미터(주행) ─────
CENTER_X      = 320
STEER_LIMIT   = 1000.0
DEFAULT_SPEED = 30.0
CURVE_SPEED   = 22.5
DEFAULT_GAIN  = 0.35 
CURVE_GAIN    = 0.70
#CURVE_GAIN    = 0.85
LANE_MIN_PIX  = 500

IMG_W, IMG_H  = 640, 480   # 처리/표시 기준 크기
# ─────────────────────────
######### safety mode parameters version 1 ########
#DEFAULT_SPEED = 30.0
#CURVE_SPEED   = 25.0
#DEFAULT_GAIN  = 0.25 
#CURVE_GAIN    = 0.55 
############ version 2 ############
# DEFAULT_SPEED = 35.0
# CURVE_SPEED   = 30.0
# DEFAULT_GAIN  = 0.25 
# CURVE_GAIN    = 0.60
######## version 3 fast but bulan ######
# DEFAULT_SPEED = 40.0
# CURVE_SPEED   = 35.0
# DEFAULT_GAIN  = 0.25 
# CURVE_GAIN    = 0.65

class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0
    def pid_control(self, cte):
        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte
        return self.kp*self.p_error + self.ki*self.i_error + self.kd*self.d_error


class LaneDetectionROS(Node):
    """Lane Detection & Motor Control (저부하 디버그)"""

    def __init__(self):
        super().__init__("lane_detection_node")
        self.bridge      = CvBridge()
        self.slidewindow = SlideWindow()
        self.pid         = PID(0.7, 0.0008, 0.15)
        #self.pid         = PID(0.0, 0.0000, 0.00)

        # ROS I/O
        self.sub_image = self.create_subscription(Image, "/image_raw", self.image_callback, 10)
        self.pub_motor = self.create_publisher(Float32MultiArray, "/lane_detection", 10)
        self.pub_lane  = self.create_publisher(Int32, "/lane_index", 10)

        # 상태
        self.cv_image: np.ndarray | None = None
        self.lane_msg  = Int32()
        self.motor_msg = Float32MultiArray()

        # 디버그 설정 파라미터
        #   - debug_enabled: 전체 on/off
        #   - debug_skip   : N 프레임마다 한 번만 창 갱신 (기본 6 → 30Hz에서 5Hz로 표시)
        #   - debug_windows: "01_Orig,04_Adaptive,08_Slide" 식으로 선택 표시 (빈 문자열이면 전체)
        self.declare_parameter('debug_enabled',True)
        self.declare_parameter('debug_skip', 1)
        self.declare_parameter('debug_windows',"01_Orig,04_Adaptive,08_Slide") #04_Adaptive,
        #self.declare_parameter('debug_windows',)

        self.debug_enabled = bool(self.get_parameter('debug_enabled').value)
        self.debug_skip    = max(1, int(self.get_parameter('debug_skip').value))
        wins_param = (self.get_parameter('debug_windows').value or '').strip()
        self.debug_filter = set([w.strip() for w in wins_param.split(',') if w.strip()])  # 비어있으면 전체

        # 프레임 스킵용 카운터
        self._frame_idx = 0

        # 창 초기화 관리(창당 최초 1회만 resizeWindow)
        self._win_inited = set()

        # 20 Hz 처리 루프
        self.create_timer(0.03, self.process_frame)
        self.get_logger().info(
            f"READY (debug_enabled={self.debug_enabled}, debug_skip={self.debug_skip}, "
            f"debug_windows={'ALL' if not self.debug_filter else ','.join(sorted(self.debug_filter))})"
        )

    # 창을 이미지 크기에 맞춰 1회만 리사이즈하고, 매 프레임은 imshow만
    def show_with_fit(self, win: str, img: np.ndarray):
        if img is None:
            return
        if (self.debug_filter and win not in self.debug_filter):
            return
        h, w = img.shape[:2]
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        if win not in self._win_inited:
            cv2.resizeWindow(win, w, h)  # 최초 1회만
            self._win_inited.add(win)
        cv2.imshow(win, img)

    def image_callback(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.cv_image = cv2.resize(img, (IMG_W, IMG_H))
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")

    def process_frame(self):
        if self.cv_image is None:
            return

        frame = self.cv_image

        # ── 파이프라인 ───────────────────────────────
        cropped = roi_for_lane(frame)
        gray, blur, adap, edged, closed = process_image(cropped)
        warped_bin = warper(adap)

        if warped_bin.dtype != np.uint8:
            warped_bin = warped_bin.astype(np.uint8)
        warped_norm = (warped_bin > 0).astype(np.uint8) if warped_bin.max() > 1 else warped_bin

        out_img, x_loc, _ = self.slidewindow.slidewindow(warped_norm)

        # lane index
        roi = warped_norm[380:, :]
        left_pix  = int(np.sum(roi[:, :IMG_W//2]  > 0))
        right_pix = int(np.sum(roi[:, IMG_W//2:] > 0))
        if right_pix > left_pix and right_pix > LANE_MIN_PIX:
            lane_idx = 1
        elif left_pix > right_pix and left_pix > LANE_MIN_PIX:
            lane_idx = 2
        else:
            lane_idx = self.lane_msg.data
        self.lane_msg.data = lane_idx
        self.pub_lane.publish(self.lane_msg)

        # control
        if x_loc is None:
            x_loc = CENTER_X
        err_px = x_loc - CENTER_X
        gain   = DEFAULT_GAIN if abs(err_px) < 20 else CURVE_GAIN   #err_px < 10
        angle  = float(np.clip((err_px)* gain, -STEER_LIMIT, STEER_LIMIT))
        #angle  = float(np.clip(0.0, -STEER_LIMIT, STEER_LIMIT))
        speed  = DEFAULT_SPEED if abs(err_px) < 20 else CURVE_SPEED  #err_px < 10
        self.get_logger().info(
            f"{x_loc}"
        )

        self.motor_msg.data = [angle, float(speed), 1.0]
        self.pub_motor.publish(self.motor_msg)

        # ── 디버그 표시(저부하) ─────────────────────
        if self.debug_enabled:
            # 프레임 스킵: 설정한 간격이 아니면 창 갱신 생략
            if (self._frame_idx % self.debug_skip) == 0:
                warped_vis = (warped_norm * 255) if warped_norm.max() <= 1 else warped_norm
                self.show_with_fit("01_Orig",      frame)
                self.show_with_fit("02_Gray",      gray)
                self.show_with_fit("03_Blur",      blur)
                self.show_with_fit("04_Adaptive",  adap)
                self.show_with_fit("05_Edged",     edged)
                self.show_with_fit("06_Closed",    closed)
                self.show_with_fit("07_WarpedBin", warped_vis)
                self.show_with_fit("08_Slide",     out_img)
                cv2.waitKey(1)
            self._frame_idx += 1


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
#각종 부드럽게 이동하는 로직 적용하기 전의 휘청휘청버전