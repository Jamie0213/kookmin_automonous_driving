
# ==============================================================
# File: avoidance_controller_heading.py
# Desc: Heading-based obstacle avoidance controller (ROS2).
#       Uses /heading (deg), /is_vehicle (Bool), /vehicle (Float32MultiArray)
# Publishes: /avoidance_motor (Float32MultiArray) = [steer, speed, flag]
#            /avoidance_state (std_msgs/String)
# FSM: IDLE → CRUISE → AVOID_INIT → PARALLEL_PASS → RECENTER → IDLE
# Notes:
#  - Latches heading & side at AVOID_INIT entry; ignores /vehicle updates during AVOID/PASS/RECENTER.
#  - side convention: vy>0 (obstacle left) → avoid to RIGHT.
#  - steer sign can be flipped with param steer_right_positive.
#  - If /vehicle not received, still works using last latched side.
# ==============================================================

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Optional, Tuple
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32, Float32MultiArray, Bool, String
from diagnostic_msgs.msg import DiagnosticArray


def qos_sensor_best_effort(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def wrap_deg180(a: float) -> float:
    a = (a + 180.0) % 360.0 - 180.0
    return a


class AvoidanceControllerHeading(Node):
    def __init__(self):
        super().__init__('avoidance_controller_heading')

        # ---- Parameters ----
        # Trigger / timing
        self.declare_parameter('trigger_dist_m', 0.9)     # start avoid when target within this distance
        self.declare_parameter('avoid_kick_deg', 15.0)    # target yaw change for avoid (deg)
        self.declare_parameter('heading_tol_deg', 2.0)    # tolerance for yaw convergence
        self.declare_parameter('parallel_hold_s', 0.60)   # time to hold parallel pass
        self.declare_parameter('recenter_hold_s', 0.25)   # time to hold near yaw_ref before finishing
        self.declare_parameter('timeout_each_s', 3.0)     # watchdog per phase

        # Gains & Limits
        self.declare_parameter('yaw2steer_gain', 18.0)    # steer units per degree
        self.declare_parameter('steer_limit', 1000.0)
        self.declare_parameter('steer_right_positive', True)

        # Speeds
        self.declare_parameter('speed_cruise', 35.0)
        self.declare_parameter('speed_avoid', 30.0)
        self.declare_parameter('speed_pass', 32.0)
        self.declare_parameter('speed_recenter', 30.0)

        # Topic names (so you can remap via params if needed)
        self.declare_parameter('topic_heading', '/heading')
        self.declare_parameter('topic_is_vehicle', '/is_vehicle')
        self.declare_parameter('topic_vehicle', '/vehicle')
        self.declare_parameter('topic_motor', '/avoidance_motor')
        self.declare_parameter('topic_state', '/avoidance_state')

        # ---- Load params ----
        self.trigger_dist = float(self.get_parameter('trigger_dist_m').value)
        self.kick_deg = float(self.get_parameter('avoid_kick_deg').value)
        self.tol_deg = float(self.get_parameter('heading_tol_deg').value)
        self.hold_pass_s = float(self.get_parameter('parallel_hold_s').value)
        self.hold_recent_s = float(self.get_parameter('recenter_hold_s').value)
        self.timeout_each_s = float(self.get_parameter('timeout_each_s').value)

        self.yaw2steer = float(self.get_parameter('yaw2steer_gain').value)
        self.steer_limit = float(self.get_parameter('steer_limit').value)
        self.right_pos = bool(self.get_parameter('steer_right_positive').value)

        self.topic_heading = str(self.get_parameter('topic_heading').value)
        self.topic_is_vehicle = str(self.get_parameter('topic_is_vehicle').value)
        self.topic_vehicle = str(self.get_parameter('topic_vehicle').value)
        self.topic_motor = str(self.get_parameter('topic_motor').value)
        self.topic_state = str(self.get_parameter('topic_state').value)

        # ---- State ----
        self.state = 'IDLE'
        self.is_vehicle: bool = False
        self.heading_deg: Optional[float] = None
        self.vehicle_vy: Optional[float] = None
        self.vehicle_dist: Optional[float] = None

        # Latched at AVOID_INIT entry
        self.yaw_ref: Optional[float] = None
        self.side: Optional[int] = None          # +1: steer right, -1: steer left (as a command direction)
        self.yaw_target: Optional[float] = None
        self.phase_start_t: Optional[float] = None

        # ---- I/O ----
        self.sub_heading = self.create_subscription(Float32, self.topic_heading, self.cb_heading, qos_sensor_best_effort(20))
        self.sub_is_vehicle = self.create_subscription(Bool, self.topic_is_vehicle, self.cb_is_vehicle, 10)
        self.sub_vehicle = self.create_subscription(Float32MultiArray, self.topic_vehicle, self.cb_vehicle, 10)

        self.pub_motor = self.create_publisher(Float32MultiArray, self.topic_motor, 10)
        self.pub_state = self.create_publisher(String, self.topic_state, 10)

        # Control loop timer (50 Hz)
        self.timer = self.create_timer(0.02, self.on_timer)
        self.get_logger().info('avoidance_controller_heading started.')

    # ---------- Callbacks ----------
    def cb_heading(self, msg: Float32):
        self.heading_deg = float(msg.data)

    def cb_is_vehicle(self, msg: Bool):
        self.is_vehicle = bool(msg.data)

    def cb_vehicle(self, msg: Float32MultiArray):
        # Expect at least [x, y, dist, ...]. Be tolerant.
        data = list(msg.data)
        if len(data) >= 2:
            self.vehicle_vy = float(data[1])
        if len(data) >= 3:
            self.vehicle_dist = float(data[2])

    # ---------- Helpers ----------
    def publish_motor(self, steer: float, speed: float, flag: int):
        m = Float32MultiArray()
        m.data = [float(steer), float(speed), float(flag)]
        self.pub_motor.publish(m)

    def set_state(self, s: str):
        if s != self.state:
            self.get_logger().info(f'State: {self.state} → {s}')
        self.state = s
        msg = String(); msg.data = s
        self.pub_state.publish(msg)
        self.phase_start_t = time.time()

    def steer_from_yaw_error(self, yaw_err_deg: float) -> float:
        # Positive yaw_err: need to rotate CCW (left). Map to steer sign according to self.right_pos.
        # We define steer_cmd > 0 means RIGHT if self.right_pos True; else LEFT.
        steer = self.yaw2steer * yaw_err_deg
        if self.right_pos:
            # yaw_err + → steer left (negative)
            steer = -steer
        # saturate
        return max(-self.steer_limit, min(self.steer_limit, steer))

    def phase_elapsed(self) -> float:
        return 0.0 if self.phase_start_t is None else (time.time() - self.phase_start_t)

    # ---------- Main control loop ----------
    def on_timer(self):
        # Require heading
        if self.heading_deg is None:
            # No heading yet → publish pass-through idle command
            self.publish_motor(0.0, float(self.get_parameter('speed_cruise').value), 0)
            return

        # FSM
        if self.state == 'IDLE':
            # Wait for target existence
            if self.is_vehicle:
                self.set_state('CRUISE')
            self.publish_motor(0.0, float(self.get_parameter('speed_cruise').value), 0)
            return

        if self.state == 'CRUISE':
            # Start avoidance if close enough; default pass-through
            dist = self.vehicle_dist if self.vehicle_dist is not None else 999.0
            if self.is_vehicle and dist <= self.trigger_dist:
                # Latch
                self.yaw_ref = self.heading_deg
                vy = self.vehicle_vy if self.vehicle_vy is not None else 0.0
                # obstacle on left (vy>0) ⇒ avoid to RIGHT ⇒ side = +1 (command direction right)
                self.side = +1 if vy > 0.0 else -1
                # Target yaw
                self.yaw_target = wrap_deg180(self.yaw_ref + self.side * self.kick_deg)
                self.set_state('AVOID_INIT')
            # CRUISE output (flag 1 to indicate we are controlling, but not yet avoiding aggressively)
            self.publish_motor(0.0, float(self.get_parameter('speed_cruise').value), 1 if self.is_vehicle else 0)
            return

        if self.state == 'AVOID_INIT':
            # Drive heading toward yaw_target
            if self.yaw_ref is None or self.yaw_target is None or self.side is None:
                # Safety fallback
                self.set_state('IDLE')
                self.publish_motor(0.0, float(self.get_parameter('speed_cruise').value), 0)
                return

            yaw_err = wrap_deg180(self.yaw_target - self.heading_deg)
            steer = self.steer_from_yaw_error(yaw_err)
            self.publish_motor(steer, float(self.get_parameter('speed_avoid').value), 1)

            # Transition conditions
            if abs(yaw_err) <= self.tol_deg:
                self.set_state('PARALLEL_PASS')
            elif self.phase_elapsed() > self.timeout_each_s:
                # Timeout safety: proceed anyway
                self.get_logger().warn('AVOID_INIT timeout → PARALLEL_PASS')
                self.set_state('PARALLEL_PASS')
            return

        if self.state == 'PARALLEL_PASS':
            # Hold near yaw_target for a short while
            yaw_err = wrap_deg180(self.yaw_target - self.heading_deg) if self.yaw_target is not None else 0.0
            steer = self.steer_from_yaw_error(yaw_err)
            self.publish_motor(steer, float(self.get_parameter('speed_pass').value), 1)

            if self.phase_elapsed() >= self.hold_pass_s:
                self.set_state('RECENTER')
            elif self.phase_elapsed() > self.timeout_each_s:
                self.get_logger().warn('PARALLEL_PASS timeout → RECENTER')
                self.set_state('RECENTER')
            return

        if self.state == 'RECENTER':
            # Go back to yaw_ref
            if self.yaw_ref is None:
                self.set_state('IDLE')
                self.publish_motor(0.0, float(self.get_parameter('speed_cruise').value), 0)
                return
            yaw_err = wrap_deg180(self.yaw_ref - self.heading_deg)
            steer = self.steer_from_yaw_error(yaw_err)
            self.publish_motor(steer, float(self.get_parameter('speed_recenter').value), 1)

            if abs(yaw_err) <= self.tol_deg and self.phase_elapsed() >= self.hold_recent_s:
                # Clear latches
                self.yaw_ref = None
                self.yaw_target = None
                self.side = None
                self.set_state('IDLE')
            elif self.phase_elapsed() > self.timeout_each_s:
                self.get_logger().warn('RECENTER timeout → IDLE')
                self.yaw_ref = None
                self.yaw_target = None
                self.side = None
                self.set_state('IDLE')
            return


# -------------------------
# Entrypoint
# -------------------------

def main():
    rclpy.init()
    node = AvoidanceControllerHeading()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()