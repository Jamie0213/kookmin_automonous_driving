# -*- coding: utf-8 -*-
import cv2, math
import numpy as np

class SlideWindow:
    """
    버전 2: 윈도우별 robust center + MAX_DX 연속성 게이트 + longest_run K 기준
    - 각 윈도우에서 중앙값/MAD + (선택)거리변환 가중치로 중심 추정
    - 창간 급변을 제한(MAX_DX)
    - 프레임 종료 후 longest_run(flags) < K 이면 해당 차선 폐기
    - 좌/우 있으면 차선폭 일관성 검사(평균/표준편차)
    - 신뢰도 낮으면 이전 EMA 유지
    """

    def __init__(self):
        self.current_line = "DEFAULT"

        # 상태
        self.prev_left_x  = None
        self.prev_right_x = None
        self.x_previous   = 320
        self.x_ema        = 320

        # ---------- 파라미터 ----------
        self.window_height = 10
        self.nwindows      = 14
        self.margin        = 20
        self.minpix        = 8
        self.min_seed_cnt  = 200
        self.circle_height = 75
        self.road_width_ratio = 0.465
        self.ema_alpha     = 0.20

        # 연속성 게이트
        self.MAX_DX        = 18  # 인접 윈도우 사이 허용 이동량(px)
        self.MIN_RUN       = 5   # 최장 연속 윈도우 개수 임계

        # 쌍 폭 체크
        self.WIDTH_MU_TOL  = 25.0  # 기대폭과의 허용 오차
        self.WIDTH_SIG_TOL = 14.0  # 폭 표준편차 허용치

        # 하단 시드 박스
        self.win_h1 = 100
        self.win_h2 = 150
        self.win_l_w_l = 150
        self.win_l_w_r = 300
        self.win_r_w_l = 480 
        self.win_r_w_r = 480 

    # ---------- 유틸 ----------
    @staticmethod
    def _longest_run(flags):
        run = mx = 0
        for f in flags:
            run = run + 1 if f else 0
            mx = max(mx, run)
        return mx

    @staticmethod
    def _robust_center(xs, ys, pred=None, distmap=None):
        """중앙값/MAD + (선택)거리변환/예측값 가중치"""
        if len(xs) == 0:
            return None
        xs = np.asarray(xs, dtype=np.int32)
        ys = np.asarray(ys, dtype=np.int32)

        med = np.median(xs)
        mad = 1.4826 * np.median(np.abs(xs - med)) + 1e-6
        keep = np.abs(xs - med) < 2.5 * mad
        xs, ys = xs[keep], ys[keep]
        if len(xs) == 0:
            return None

        w = np.ones_like(xs, dtype=np.float32)
        if distmap is not None:
            w *= (1.0 + distmap[ys, xs])
        if pred is not None:
            w *= np.exp(-((xs - pred) ** 2) / (2 * (25.0**2)))

        try:
            return int(np.average(xs, weights=w))
        except ZeroDivisionError:
            return int(np.median(xs))

    # ---------- 메인 ----------
    def slidewindow(self, img):
        """
        img: 2D binary (0/1 or 0/255). BEV 권장
        """
        out_img = np.dstack((img, img, img))
        if out_img.max() <= 1: out_img = out_img * 255
        h, w = img.shape[:2]

        # 거리변환(얇은 노이즈 < 굵은 차선)
        dist_in = (img.astype(np.uint8) > 0).astype(np.uint8) * 255
        distmap = cv2.distanceTransform(dist_in, cv2.DIST_L2, 3)

        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0], dtype=np.int32)
        nonzerox = np.array(nonzero[1], dtype=np.int32)

        # 시각화
        cv2.line(out_img, (0, self.circle_height), (w, self.circle_height), (0,120,120), 1)
        cv2.polylines(out_img, [np.array([[self.win_l_w_l,self.win_h2],[self.win_l_w_l,self.win_h1],
                                          [self.win_l_w_r,self.win_h1],[self.win_l_w_r,self.win_h2]])], False, (0,255,0), 1)
        cv2.polylines(out_img, [np.array([[self.win_r_w_l,self.win_h2],[self.win_r_w_l,self.win_h1],
                                          [self.win_r_w_r,self.win_h1],[self.win_r_w_r,self.win_h2]])], False, (255,0,0), 1)

        # 시드
        Lseed = ((nonzerox >= self.win_l_w_l) & (nonzerox <= self.win_l_w_r) &
                 (nonzeroy >  self.win_h1)     & (nonzeroy <= self.win_h2)).nonzero()[0]
        Rseed = ((nonzerox >= self.win_r_w_l) & (nonzerox <= self.win_r_w_r) &
                 (nonzeroy >  self.win_h1)     & (nonzeroy <= self.win_h2)).nonzero()[0]

        xL = int(np.mean(nonzerox[Lseed])) if len(Lseed) > self.min_seed_cnt else None
        yL = int(np.max(nonzeroy[Lseed]))  if len(Lseed) > self.min_seed_cnt else None
        xR = int(np.mean(nonzerox[Rseed])) if len(Rseed) > self.min_seed_cnt else None
        yR = int(np.max(nonzeroy[Rseed]))  if len(Rseed) > self.min_seed_cnt else None

        # 누적
        left_lane_inds, right_lane_inds = [], []
        left_hits_flags, right_hits_flags = [], []
        x_prev_left = x_prev_right = None

        pL = pR = None  # 중간중간 예측에 쓸 수 있음

        for k in range(self.nwindows):
            y_low  = lambda y0: int(y0 - (k + 1) * self.window_height)
            y_high = lambda y0: int(y0 - k * self.window_height)

            # LEFT
            if xL is not None and yL is not None:
                xl = int(xL - self.margin); xh = int(xL + self.margin)
                yl = y_low(yL);            yh = y_high(yL)
                cv2.rectangle(out_img, (xl, yl), (xh, yh), (0,255,0), 1)
                good = ((nonzeroy >= yl) & (nonzeroy < yh) & (nonzerox >= xl) & (nonzerox < xh)).nonzero()[0]
                left_lane_inds.extend(good)

                # robust center with gate
                y_c  = (yl + yh) // 2
                pred = int(np.polyval(pL, y_c)) if pL is not None else None
                x_meas = self._robust_center(nonzerox[good], nonzeroy[good], pred=pred, distmap=distmap)

                if x_meas is not None:
                    if x_prev_left is not None and abs(x_meas - x_prev_left) > self.MAX_DX:
                        xL = pred if pred is not None else x_prev_left
                        left_hits_flags.append(False)  # 급변 → 측정 버림
                    else:
                        xL = x_meas
                        left_hits_flags.append(True)
                        x_prev_left = xL
                else:
                    left_hits_flags.append(False)
            else:
                left_hits_flags.append(False)

            # RIGHT
            if xR is not None and yR is not None:
                xl = int(xR - self.margin); xh = int(xR + self.margin)
                yl = y_low(yR);            yh = y_high(yR)
                cv2.rectangle(out_img, (xl, yl), (xh, yh), (255,0,0), 1)
                good = ((nonzeroy >= yl) & (nonzeroy < yh) & (nonzerox >= xl) & (nonzerox < xh)).nonzero()[0]
                right_lane_inds.extend(good)

                y_c  = (yl + yh) // 2
                pred = int(np.polyval(pR, y_c)) if pR is not None else None
                x_meas = self._robust_center(nonzerox[good], nonzeroy[good], pred=pred, distmap=distmap)

                if x_meas is not None:
                    if x_prev_right is not None and abs(x_meas - x_prev_right) > self.MAX_DX:
                        xR = pred if pred is not None else x_prev_right
                        right_hits_flags.append(False)
                    else:
                        xR = x_meas
                        right_hits_flags.append(True)
                        x_prev_right = xR
                else:
                    right_hits_flags.append(False)
            else:
                right_hits_flags.append(False)

            # 중간 적합(선택) : inlier를 조금씩 모았으면 업데이트
            if len(left_lane_inds) > 60:
                y = nonzeroy[left_lane_inds]; x = nonzerox[left_lane_inds]
                pL = np.polyfit(y, x, 2)
            if len(right_lane_inds) > 60:
                y = nonzeroy[right_lane_inds]; x = nonzerox[right_lane_inds]
                pR = np.polyfit(y, x, 2)

        # 프레임 끝: 최장연속(run) 기준
        accept_left  = self._longest_run(left_hits_flags)  >= self.MIN_RUN and len(left_lane_inds)  >= 40
        accept_right = self._longest_run(right_hits_flags) >= self.MIN_RUN and len(right_lane_inds) >= 40

        # 최종 poly
        if accept_left:
            y = nonzeroy[left_lane_inds]; x = nonzerox[left_lane_inds]
            pL = np.polyfit(y, x, 2)
        else:
            pL = None
        if accept_right:
            y = nonzeroy[right_lane_inds]; x = nonzerox[right_lane_inds]
            pR = np.polyfit(y, x, 2)
        else:
            pR = None

        # 쌍 폭 일관성 검사
        if pL is not None and pR is not None:
            ys = np.linspace(max(self.win_h1, self.circle_height-60),
                             min(self.win_h2 + self.nwindows*self.window_height, h-1), 8)
            width_samples = np.polyval(pR, ys) - np.polyval(pL, ys)
            mu, sigma = float(np.mean(width_samples)), float(np.std(width_samples))
            expected = w * self.road_width_ratio
            if abs(mu - expected) > self.WIDTH_MU_TOL or sigma > self.WIDTH_SIG_TOL:
                # 더 불안정해 보이는 쪽 제거(표본 수/잔차로 결정)
                # 간단 버전: 표본 수가 적은 쪽을 버림
                if len(left_lane_inds) >= len(right_lane_inds):
                    pR = None
                else:
                    pL = None

        # x_location 결정
        expected_half = int(w * self.road_width_ratio * 0.5)
        have_left  = pL is not None
        have_right = pR is not None

        if have_left and have_right:
            xL_now = int(np.polyval(pL, self.circle_height))
            xR_now = int(np.polyval(pR, self.circle_height))
            x_center = (xL_now + xR_now) // 2 - 90
            self.current_line = "BOTH"
        elif have_left:
            x_center = int(np.polyval(pL, self.circle_height)) + expected_half - 90
            self.current_line = "LEFT"
        elif have_right:
            x_center = int(np.polyval(pR, self.circle_height)) - expected_half - 90
            self.current_line = "RIGHT"
        else:
            x_center = self.x_previous
            self.current_line = "DEFAULT"

        # EMA + 상태 업데이트
        self.x_ema = int((1-self.ema_alpha)*self.x_ema + self.ema_alpha*x_center)
        x_location = int(self.x_ema)
        if pL is not None: self.prev_left_x  = int(np.polyval(pL, self.circle_height))
        if pR is not None: self.prev_right_x = int(np.polyval(pR, self.circle_height))
        self.x_previous = x_location

        cv2.circle(out_img, (x_location, self.circle_height), 10, (0,255,255), -1)
        return out_img, x_location, self.current_line