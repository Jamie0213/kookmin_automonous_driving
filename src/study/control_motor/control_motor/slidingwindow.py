import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import *

# ────────────────────────────────────────────────
# SlideWindow 클래스
# ㆍ전처리된 이진 이미지를 입력받아 슬라이딩‑윈도우 기법을 사용하여 차량 중심점을 추정하고, 디버깅용 시각화 이미지를 반환합니다.
# ㆍ추정된 중심점(x_location)은 이후 lane_detection.py에서 조향 제어에 사용됩니다.
# ────────────────────────────────────────────────

class SlideWindow:
    def __init__(self):
        # 상태 변수 초기화
        self.current_line = "DEFAULT"
        self.left_fit  = None
        self.right_fit = None
        self.leftx  = None
        self.rightx = None
        self.lhd = 240
        self.left_cnt  = 25
        self.right_cnt = 25
        self.x_previous = 320  # 미검출 시 사용할 이전 중심값

    def slidewindow(self, img):
        """슬라이딩‑윈도우로 차선 중심 추정
        img : 1‑채널 이진 이미지 (np.uint8)
        roi_flag : (예비) ROI 스위치 – 현재 미사용
        반환 : (컬러 시각화 이미지, 중심 x 좌표, 기준 차선 상태)
        """
        # 기본 중심값
        x_location = 320

        # 시각화용 컬러 이미지, 화면 크기
        out_img = np.dstack((img, img, img)) * 255  # 흑백→BGR
        height, width = img.shape

        # 슬라이딩 윈도우 파라미터
        window_height = 15
        nwindows = 30
        margin = 40
        minpix = 0

        # 스타트 ROI 설정
        win_h1, win_h2 = 420, 480
        win_l_w_l, win_l_w_r = 125, 245   # 왼쪽 x 범위
        win_r_w_l, win_r_w_r = 395, 515   # 오른쪽 x 범위
        # win_l_w_l, win_l_w_r = 105, 265   # 왼쪽 x 범위
        # win_r_w_l, win_r_w_r = 375, 535   # 오른쪽 x 범위
        circle_height = 200
        road_width = 0.47
        half_road_width = 0.5 * road_width

        # ROI 윤곽선(디버깅)
        cv2.polylines(out_img, [np.array([[win_l_w_l, win_h2], [win_l_w_l, win_h1], [win_l_w_r, win_h1], [win_l_w_r, win_h2]], np.int32)], False, (0,255,0), 1)
        cv2.polylines(out_img, [np.array([[win_r_w_l, win_h2], [win_r_w_l, win_h1], [win_r_w_r, win_h1], [win_r_w_r, win_h2]], np.int32)], False, (255,0,0), 1)
        cv2.line(out_img, (0, circle_height), (width, circle_height), (0,120,120), 1)

        # 이미지 내 유효 픽셀 좌표
        nonzeroy, nonzerox = img.nonzero()
        nonzeroy = np.array(nonzeroy)
        nonzerox = np.array(nonzerox)

        # 스타트 ROI 픽셀 인덱스
        good_left_inds = ((nonzerox >= win_l_w_l) & (nonzerox <= win_l_w_r) & (nonzeroy > win_h1) & (nonzeroy <= win_h2)).nonzero()[0]
        good_right_inds = ((nonzerox >= win_r_w_l) & (nonzerox <= win_r_w_r) & (nonzeroy > win_h1) & (nonzeroy <= win_h2)).nonzero()[0]

        # 기준 차선 결정
        y_current = height - 1
        x_current = None
        if len(good_left_inds) > len(good_right_inds):
            line_flag = 1  # 왼쪽 기준
            x_current = int(np.mean(nonzerox[good_left_inds]))
        elif len(good_left_inds) < len(good_right_inds):
            line_flag = 2  # 오른쪽 기준
            x_current = int(np.mean(nonzerox[good_right_inds]))
        else:
            line_flag = 3  # 양쪽 모두 부족
            self.current_line = "MID"

        # 시작 ROI 픽셀 시각화
        if line_flag == 1:
            for idx in good_left_inds:
                cv2.circle(out_img, (nonzerox[idx], nonzeroy[idx]), 1, (0,255,0), -1)
        elif line_flag == 2:
            for idx in good_right_inds:
                cv2.circle(out_img, (nonzerox[idx], nonzeroy[idx]), 1, (255,0,0), -1)

        # 슬라이딩 윈도우 탐색
        for window in range(nwindows):
            if line_flag == 1:  # 왼쪽 기준
                win_y_low  = y_current - (window + 1) * window_height
                win_y_high = y_current - window * window_height
                win_x_low  = x_current - margin
                win_x_high = x_current + margin

                cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0,255,0), 1)
                cv2.rectangle(out_img, (win_x_low + int(width * road_width), win_y_low),
                                          (win_x_high + int(width * road_width), win_y_high), (255,0,0), 1)

                good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                if len(good_left_inds) > minpix:
                    x_current = int(np.mean(nonzerox[good_left_inds]))
                elif len(self.left_fit or []):
                    x_current = int(np.polyval(self.left_fit, win_y_high))

                if circle_height - 10 <= win_y_low < circle_height + 10:
                    x_location = int(x_current + width * half_road_width)
                    cv2.circle(out_img, (x_location, circle_height), 10, (0,0,255), 5)

            elif line_flag == 2:  # 오른쪽 기준
                win_y_low  = y_current - (window + 1) * window_height
                win_y_high = y_current - window * window_height
                win_x_low  = x_current - margin
                win_x_high = x_current + margin

                cv2.rectangle(out_img, (win_x_low - int(width * road_width), win_y_low),
                                          (win_x_high - int(width * road_width), win_y_high), (0,255,0), 1)
                cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255,0,0), 1)

                good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                if len(good_right_inds) > minpix:
                    x_current = int(np.mean(nonzerox[good_right_inds]))
                elif len(self.right_fit or []):
                    x_current = int(np.polyval(self.right_fit, win_y_high))

                if circle_height - 10 <= win_y_low < circle_height + 10:
                    x_location = int(x_current - width * half_road_width)
                    cv2.circle(out_img, (x_location, circle_height), 10, (0,0,255), 5)

            else:  # 차선 모두 미검출
                x_location = self.x_previous
                cv2.circle(out_img, (x_location, circle_height), 10, (0,0,255), 5)

            # 추적 실패 시 이전 값 사용
            if x_location == 320:
                x_location = self.x_previous
                cv2.circle(out_img, (x_location, circle_height), 10, (0,0,255), 5)

            self.x_previous = x_location

        return out_img, x_location, self.current_line
