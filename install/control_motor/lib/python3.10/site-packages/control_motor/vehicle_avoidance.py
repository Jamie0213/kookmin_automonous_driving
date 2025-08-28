# ==============================================================
# File: avoidance_controller.py (time-based + x_location gate)
# Desc: /heading 없이 시간 기반 회피. 커밋은 /vehicle 콜백에서 '이벤트 드리븐'
#       x_location(min/max) 창 안에서만 flag=1 퍼블리시
#       FSM: IDLE ↔ AVOID_INIT
#       출력: /avoidance_motor(Float32MultiArray) = [steer, speed, flag]
#
# 변경 요약
#  - yaw 의존 제거(/heading 구독 제거)
#  - AVOID_INIT 내 페이즈 전환을 시간 기반(phase0_time_sec / phase1_time_sec)으로 수행
#  - 커밋 게이트: is_vehicle=True, v_count>0, v_dist<=commit_d, 쿨다운 아님,
#                그리고 /vehicle이 이번 라운드 /is_vehicle(True) 이후에 들어온 샘플
#  - 회피 종료 시 /vehicle 상태 무효화(_have_vehicle=False, v_dist=inf)
#  - ★ x_location(min/max) 파라미터 추가: 창 안에서만 flag=1 (그 외 flag=0)
# ==============================================================

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Optional, Tuple
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32MultiArray, String, Bool, Float32


# ---------------- QoS ----------------
def qos_best_effort(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )

def qos_reliable(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )

class AvoidanceController(Node):
    IDLE, AVOID_INIT = range(2)

    def __init__(self):
        super().__init__('avoidance_controller')
        # ------- 파라미터 -------
        self.declare_parameter('motor_topic', '/avoidance_motor')
        self.declare_parameter('lane_topic', '/lane_detection')
        self.declare_parameter('vehicle_topic', '/vehicle')
        self.declare_parameter('is_vehicle_topic', '/is_vehicle')
        self.declare_parameter('x_location_topic', '/x_location')

        self.declare_parameter('commit_distance', 1.0)        # 회피 진입 거리 한계(기본 1.0 m)
        self.declare_parameter('steer_right_is_positive', True)  # 우조향 +
        self.declare_parameter('steer_fast_mag', 50.0)        # 초기 킥 크기 (LIMIT=100 기준)
        self.declare_parameter('avoid_speed', 35.0)           # AVOID_INIT 중 속도 제한
        self.declare_parameter('steer_limit', 100.0)
        self.declare_parameter('publish_rate_hz', 70.0)

        # 시간 기반 파라미터
        self.declare_parameter('phase0_time_sec', 0.35)       # kick 구간 지속 시간
        self.declare_parameter('phase1_time_sec', 1.70)       # 역조향 구간 지속 시간
        self.declare_parameter('phase0_steer_bias', 0.0)      # base + kick + bias0
        self.declare_parameter('phase1_steer_bias', -20.0)    # base - kick + bias1

        # stale 한도 / 쿨다운
        self.declare_parameter('vehicle_stale_sec', 0.6)
        self.declare_parameter('is_vehicle_stale_sec', 0.6)
        self.declare_parameter('xloc_stale_sec', 0.6)         # ★ x_location 신선도 한도
        self.declare_parameter('cooldown_sec', 2.0)

        # 방어적 신선도 파라미터(이벤트 드리븐이라 거의 0에 가까움)
        self.declare_parameter('vehicle_fresh_max_age_sec', 0.15)

        # ★ x_location 게이트 창
        self.declare_parameter('xloc_min', 280.0)  # 픽셀 (IMG_W=640 기준)
        self.declare_parameter('xloc_max', 360.0)

        # ------- 파라미터 값 캐시 -------
        self.motor_topic = str(self.get_parameter('motor_topic').value)
        self.lane_topic = str(self.get_parameter('lane_topic').value)
        self.vehicle_topic = str(self.get_parameter('vehicle_topic').value)
        self.is_vehicle_topic = str(self.get_parameter('is_vehicle_topic').value)
        self.x_location_topic = str(self.get_parameter('x_location_topic').value)

        self.commit_d = float(self.get_parameter('commit_distance').value)
        self.right_pos = bool(self.get_parameter('steer_right_is_positive').value)
        self.steer_fast_mag = float(self.get_parameter('steer_fast_mag').value)
        self.avoid_speed = float(self.get_parameter('avoid_speed').value)
        self.steer_limit = float(self.get_parameter('steer_limit').value)

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.dt = 1.0 / max(1.0, rate_hz)

        self.phase0_time = float(self.get_parameter('phase0_time_sec').value)
        self.phase1_time = float(self.get_parameter('phase1_time_sec').value)
        self.phase0_bias = float(self.get_parameter('phase0_steer_bias').value)
        self.phase1_bias = float(self.get_parameter('phase1_steer_bias').value)

        self.vehicle_stale_sec = float(self.get_parameter('vehicle_stale_sec').value)
        self.is_vehicle_stale_sec = float(self.get_parameter('is_vehicle_stale_sec').value)
        self.xloc_stale_sec = float(self.get_parameter('xloc_stale_sec').value)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)
        self.vehicle_fresh_max_age = float(self.get_parameter('vehicle_fresh_max_age_sec').value)

        self.xloc_min = float(self.get_parameter('xloc_min').value)
        self.xloc_max = float(self.get_parameter('xloc_max').value)

        # ------- ROS I/O -------
        self.sub_lane = self.create_subscription(Float32MultiArray, self.lane_topic, self.on_lane, qos_reliable(10))
        self.sub_vehicle = self.create_subscription(Float32MultiArray, self.vehicle_topic, self.on_vehicle, qos_best_effort(10))
        self.sub_is_vehicle = self.create_subscription(Bool, self.is_vehicle_topic, self.on_is_vehicle, qos_best_effort(10))
        self.sub_xloc = self.create_subscription(Float32, self.x_location_topic, self.on_x_location, qos_best_effort(10))

        self.pub_motor = self.create_publisher(Float32MultiArray, self.motor_topic, qos_reliable(10))
        self.pub_mode  = self.create_publisher(String, '/avoidance_mode', qos_best_effort(5))

        # ------- 상태 -------
        self.mode = self.IDLE
        self.mode_name = 'IDLE'

        # lane 최신값
        self.lane_steer: float = 0.0
        self.lane_speed: float = 0.0
        self._lane_flag: float = 1.0
        self._have_lane = False

        # /vehicle 최신값
        self.v_x: float = 0.0
        self.v_y: float = 0.0
        self.v_dist: float = float('inf')
        self.v_ang: float = 0.0
        self.v_count: int = 0
        self._have_vehicle = False
        self._t_vehicle = self.get_clock().now()

        # /is_vehicle 최신값
        self.is_vehicle: bool = False
        self._have_is_vehicle: bool = False
        self._t_is_vehicle = self.get_clock().now()

        # /x_location 최신값
        self.x_location: Optional[float] = None
        self._have_xloc: bool = False
        self._t_xloc = self.get_clock().now()

        # AVOID_INIT 래치/변수
        self.base_steer: float = 0.0
        self.base_speed: float = 0.0
        self.side: Optional[str] = None  # 'left' or 'right'
        self._phase = 0  # 0: base+kick, 1: base-kick
        self._kick = 0.0
        self._sign = +1.0  # left=+1, right=-1
        self._t_phase_start = None

        # 쿨다운
        self._t_cooldown_start = None

        # 디버그 스로틀
        self._t_last_block_log = self.get_clock().now()

        # 루프 타이머
        self.create_timer(self.dt, self.loop)
        self.get_logger().info("AvoidanceController(TIME-BASED,event-driven,NO-CRUISE,xloc-gated) started")

    # ---------- 콜백 ----------
    def on_lane(self, msg: Float32MultiArray):
        try:
            d = msg.data
            if len(d) >= 2:
                self.lane_steer = float(d[0])
                self.lane_speed = float(d[1])
                self._lane_flag = float(d[2]) if len(d) >= 3 else 1.0
                self._have_lane = True
        except Exception as e:
            self.get_logger().warn(f"lane parse error: {e}")

    def on_vehicle(self, msg: Float32MultiArray):
        """유연 파싱: len>=3: x,y,dist / len>=4: bearing / len>=5: count"""
        try:
            d = list(map(float, msg.data))
            if len(d) >= 3:
                self.v_x, self.v_y, self.v_dist = d[0], d[1], d[2]
                if len(d) >= 4:
                    self.v_ang = d[3]
                if len(d) >= 5:
                    self.v_count = int(d[4])
                else:
                    self.v_count = max(1, getattr(self, 'v_count', 0))
                self._have_vehicle = True
                self._t_vehicle = self.get_clock().now()
            else:
                self._have_vehicle = False
                return
        except Exception as e:
            self.get_logger().warn(f"vehicle parse error: {e}")
            return

        # ===== 커밋을 이벤트 드리븐으로: 이 콜백에서 즉시 판단 =====
        self._maybe_commit_from_vehicle()

    def on_is_vehicle(self, msg: Bool):
        self.is_vehicle = bool(msg.data)
        self._have_is_vehicle = True
        self._t_is_vehicle = self.get_clock().now()

    def on_x_location(self, msg: Float32):
        try:
            self.x_location = float(msg.data)
            self._have_xloc = True
            self._t_xloc = self.get_clock().now()
        except Exception as e:
            self.get_logger().warn(f"x_location parse error: {e}")

    # ---------- 헬퍼 ----------
    def _refresh_staleness(self):
        now = self.get_clock().now()
        if self._have_vehicle and (now - self._t_vehicle).nanoseconds * 1e-9 > self.vehicle_stale_sec:
            self._have_vehicle = False
        if self._have_is_vehicle and (now - self._t_is_vehicle).nanoseconds * 1e-9 > self.is_vehicle_stale_sec:
            self._have_is_vehicle = False
        if self._have_xloc and (now - self._t_xloc).nanoseconds * 1e-9 > self.xloc_stale_sec:
            self._have_xloc = False

    def _gate_vehicle_present(self) -> bool:
        return self._have_is_vehicle and self.is_vehicle

    def _gate_xloc_in_window(self) -> bool:
        if not self._have_xloc or self.x_location is None:
            return False
        return (self.xloc_min <= self.x_location <= self.xloc_max)

    def _flag_on(self) -> bool:
        # 둘 다 만족할 때만 flag=1
        return self._gate_vehicle_present() and self._gate_xloc_in_window()

    def _cooldown_active(self) -> bool:
        if self._t_cooldown_start is None:
            return False
        now = self.get_clock().now()
        return (now - self._t_cooldown_start).nanoseconds * 1e-9 < self.cooldown_sec

    def _throttled_block_log(self, reason: str, period_sec: float = 0.5):
        now = self.get_clock().now()
        if (now - self._t_last_block_log).nanoseconds * 1e-9 >= period_sec:
            self.get_logger().info(reason)
            self._t_last_block_log = now

    # ★ 이벤트 드리븐 커밋 로직
    def _maybe_commit_from_vehicle(self):
        # 이미 회피 중이면 무시
        if self.mode == self.AVOID_INIT:
            return

        # 게이트 꺼짐, 쿨다운 중 → 불가
        if not self._gate_vehicle_present():
            return
        if self._cooldown_active():
            self._throttled_block_log("[avoid] COOLDOWN active: avoidance disabled")
            return

        # /vehicle 유효성
        if not self._have_vehicle or self.v_count <= 0:
            self._throttled_block_log("[avoid] BLOCKED: /vehicle invalid (no cluster)")
            return

        # 거리 조건(기본 1.0m 이내)
        if self.v_dist > self.commit_d:
            return

        # /is_vehicle(True) 이후에 들어온 샘플만 허용
        if self._t_vehicle < self._t_is_vehicle:
            self._throttled_block_log("[avoid] WAIT: need /vehicle AFTER /is_vehicle(True)")
            return

        # 신선도 체크(방어적)
        now = self.get_clock().now()
        age = (now - self._t_vehicle).nanoseconds * 1e-9
        if age > self.vehicle_fresh_max_age:
            self._throttled_block_log(f"[avoid] BLOCKED: /vehicle too old ({age*1000:.0f}ms)")
            return

        # 모든 게이트 통과 → 즉시 회피 진입
        self._enter_avoid_init()


    # ---------- 루프(간단 pass-through + AVOID_INIT 제어) ----------
    def loop(self):
        self._refresh_staleness()

        if self.mode == self.IDLE:
            self.mode_name = 'IDLE'
            steer, speed = self._idle_control()
            self._publish(steer, speed, self._flag_on())

        else:  # AVOID_INIT
            self.mode_name = 'AVOID_INIT'
            steer, speed, done = self._avoid_init_control()
            if done:
                self.get_logger().info("AVOID_INIT done → IDLE (start cooldown)")
                self._t_cooldown_start = self.get_clock().now()
                self._reset_to_idle()
            self._publish(steer, speed, self._flag_on())

        # 모드명 퍼블리시(디버그)
        m = String(); m.data = self.mode_name
        self.pub_mode.publish(m)

    # ---------- 모드 구현 ----------
    def _idle_control(self) -> Tuple[float, float]:
        """IDLE: lane pass-through"""
        steer = self.lane_steer if self._have_lane else 0.0
        speed = self.lane_speed if self._have_lane else 0.0
        return steer, speed

    def _enter_avoid_init(self):
        # 좌/우 판단: y>0 → 좌측, y<0 → 우측  (side만 래치)
        self.side = 'left' if self.v_y > 0.0 else 'right'
        self.get_logger().info(
            f"[veh] commit x={self.v_x:.2f} y={self.v_y:.2f} dist={self.v_dist:.2f} "
            f"ang={self.v_ang:.2f} cnt={self.v_count} → side={self.side}"
        )

        # 조향 부호(좌측 물체면 우로 피함)
        sign_right = +1.0 if self.right_pos else -1.0
        sign_left  = -sign_right
        self._kick = (sign_right if self.side == 'left' else sign_left) * self.steer_fast_mag
        self._sign = -1.0 if self.side == 'left' else +1.0  # 정보용

        # lane 래치
        self.base_steer = self.lane_steer if self._have_lane else 0.0
        self.base_speed = self.lane_speed if self._have_lane else 0.0

        # 페이즈 초기화
        self._phase = 0
        self._t_phase_start = self.get_clock().now()
        self.mode = self.AVOID_INIT
        self.get_logger().info(
            f"Enter AVOID_INIT(TIME): side={self.side}, base=[{self.base_steer:.1f},{self.base_speed:.1f}], "
            f"kick={self._kick:.1f}, t0={self.phase0_time:.2f}s, t1={self.phase1_time:.2f}s"
        )

    def _elapsed_since_phase_start(self) -> float:
        if self._t_phase_start is None:
            return 0.0
        now = self.get_clock().now()
        return (now - self._t_phase_start).nanoseconds * 1e-9

    def _avoid_init_control(self) -> Tuple[float, float, bool]:
        """return (steer, speed, done). ※진행 중엔 /is_vehicle, /vehicle 미사용"""
        t = self._elapsed_since_phase_start()

        if self._phase == 0:
            # 빠르게 꺾기: base + kick (+ bias)
            steer_cmd = self._kick + self.phase0_bias
            speed_cmd = min(self.base_speed, self.avoid_speed)

            # 시간 경과로 페이즈 전환
            if t >= self.phase0_time:
                self._phase = 1
                self._t_phase_start = self.get_clock().now()
            return steer_cmd, speed_cmd, False

        else:
            # 동일 크기의 역조향: base - kick (+ bias)
            steer_cmd =  - self._kick + self.phase1_bias
            speed_cmd = min(self.base_speed, self.avoid_speed)

            # 시간 경과로 완료
            done = t >= self.phase1_time
            return steer_cmd, speed_cmd, done

    # ---------- 공통 ----------
    def _publish(self, steer: float, speed: float, flag_on: bool):
        steer = float(np.clip(steer, -self.steer_limit, self.steer_limit))
        msg = Float32MultiArray()
        msg.data = [steer, float(speed), 1.0 if flag_on else 0.0]
        self.pub_motor.publish(msg)

    def _reset_to_idle(self):
        self.mode = self.IDLE
        self.mode_name = 'IDLE'
        self.side = None
        self._phase = 0
        self._kick = 0.0
        self._sign = +1.0
        self._t_phase_start = None
        # 다음 회피는 반드시 새 /vehicle로만 판단되도록 강제 무효화
        self._have_vehicle = False
        self.v_dist = float('inf')


def main():
    rclpy.init()
    node = AvoidanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
