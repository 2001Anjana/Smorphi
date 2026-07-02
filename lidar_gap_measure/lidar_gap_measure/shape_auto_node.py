#!/usr/bin/env python3
import time

from std_srvs.srv import SetBool

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool
from geometry_msgs.msg import Twist


class ShapeAutoNode(Node):
    """
    Shape transform driven by FRONT gap + BACK escape, with a front STOP guard.

      STOP : front_distance < front_stop_distance
             -> wall directly ahead ("0 gap"): publish zero cmd_vel, hold,
                and do NOT transform while blocked.

      O→I  : front_gap_min < threshold  (a narrow but passable gap is within the
             front region, x in 0..0.40 m).  A wall BEHIND the robot can no
             longer cause this, because the back of the corridor is ignored for
             O→I.

      I→O  : front_gap_min  >= front_clear_gap   (open ahead, no narrow pinch)
             AND
             back_gap_min   >= back_clear_gap    (open behind, no narrow pinch)
             -> the robot has FULLY escaped the gap, so it can open up again.

             *** This is the important fix. ***
             The old code used `back_distance`, a straight-back ±5° cone that
             shot straight down the corridor and read large clearance (or inf)
             even while the robot's rear was still flanked by the gap walls.
             That made the robot re-open (I→O) BEFORE it had physically cleared
             the gap.  We now use `back_gap_min`, the actual GAP WIDTH between
             the left/right walls behind the robot.  The gap is still seen as
             "narrow" until it leaves the ~60 cm rear window, so the robot stays
             in I until it has truly escaped.  Both the front and back must read
             open (>= 50/60 cm) before re-opening.

    Subscribes:
      front_gap_min  (Float32) - narrowest gap in the front region (99.0 = open)
      back_gap_min   (Float32) - narrowest gap in the back  region (99.0 = open)
      front_distance (Float32) - lidar->front distance (inf = clear); STOP guard

    Publishes:
      shape_need (Int32) -> 1 = O→I, 0 = I→O
      path_blocked (Bool) -> True while a wall blocks the front
      cmd_vel (Twist)     -> zero velocity while blocked (optional)
    """
    def __init__(self):
        super().__init__('shape_auto_node')

        # --- Topics ---
        self.declare_parameter('front_gap_topic', 'front_gap_min')
        self.declare_parameter('back_gap_topic',  'back_gap_min')
        self.declare_parameter('front_dist_topic', 'front_distance')
        self.declare_parameter('clearance_topic', 'left_back_clearance_min')
        self.declare_parameter('shape_topic', 'shape_need')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('blocked_topic', 'path_blocked')

        # --- Thresholds ---
        self.declare_parameter('threshold', 0.50)            # narrow-gap (O→I)
        self.declare_parameter('front_clear_gap', 0.50)      # front open  (I→O)
        self.declare_parameter('back_clear_gap', 0.60)       # back open   (I→O)
        self.declare_parameter('required_clearance', 0.40)   # left-back quarter-disc
        self.declare_parameter('front_stop_distance', 0.15)  # wall-ahead -> STOP
        self.declare_parameter('stable_samples', 5)
        self.declare_parameter('cooldown_s', 2.0)
        self.declare_parameter('publish_stop_cmd_vel', True)

        self.front_gap_topic  = self.get_parameter('front_gap_topic').value
        self.back_gap_topic   = self.get_parameter('back_gap_topic').value
        self.front_dist_topic = self.get_parameter('front_dist_topic').value
        self.clearance_topic  = self.get_parameter('clearance_topic').value
        self.shape_topic      = self.get_parameter('shape_topic').value
        self.cmd_vel_topic    = self.get_parameter('cmd_vel_topic').value
        self.blocked_topic    = self.get_parameter('blocked_topic').value

        self.threshold       = float(self.get_parameter('threshold').value)
        self.front_clear_gap = float(self.get_parameter('front_clear_gap').value)
        self.back_clear_gap  = float(self.get_parameter('back_clear_gap').value)
        self.required_clr    = float(self.get_parameter('required_clearance').value)
        self.front_stop      = float(self.get_parameter('front_stop_distance').value)
        self.stable_n        = int(self.get_parameter('stable_samples').value)
        self.cooldown        = float(self.get_parameter('cooldown_s').value)
        self.pub_stop_vel    = bool(self.get_parameter('publish_stop_cmd_vel').value)

        # --- State ---
        self.current_shape = None   # None=unknown, 0=O, 1=I
        self.last_cmd_time = 0.0
        self.count_to_i = 0
        self.count_to_o = 0
        self.blocked = False

        self.latest_front_gap  = None
        self.latest_back_gap   = None
        self.latest_front_dist = None
        self.latest_clearance  = None

        self.enabled = False

        # --- ROS interfaces ---
        self.sub_fgap = self.create_subscription(
            Float32, self.front_gap_topic, self.on_front_gap, 10)
        self.sub_bgap = self.create_subscription(
            Float32, self.back_gap_topic, self.on_back_gap, 10)
        self.sub_fdist = self.create_subscription(
            Float32, self.front_dist_topic, self.on_front_dist, 10)
        self.sub_clr = self.create_subscription(
            Float32, self.clearance_topic, self.on_clearance, 10)

        self.pub_shape   = self.create_publisher(Int32, self.shape_topic, 10)
        self.pub_blocked = self.create_publisher(Bool, self.blocked_topic, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.srv = self.create_service(SetBool, '/shape_auto/enable', self.on_enable)

        # Keep republishing the stop command at 10 Hz while blocked.
        self.create_timer(0.1, self.stop_tick)
        # Debug print every 2 s.
        self.create_timer(2.0, self.debug_log)

        self.get_logger().info(
            f"ShapeAutoNode ready | O→I front_gap<{self.threshold}m | "
            f"I→O front>={self.front_clear_gap}m AND back>={self.back_clear_gap}m | "
            f"clearance gate (both ways) left-back>={self.required_clr}m | "
            f"STOP front<{self.front_stop}m | "
            f"stable={self.stable_n} | cooldown={self.cooldown}s"
        )

    # ------------------------------------------------------------------ #
    #  Debug
    # ------------------------------------------------------------------ #
    def debug_log(self):
        if not self.enabled:
            return
        shape = ('O' if self.current_shape == 0
                 else 'I' if self.current_shape == 1 else '?')
        self.get_logger().info(
            f"[DEBUG] shape={shape} blocked={self.blocked} | "
            f"front_gap={self.latest_front_gap} back_gap={self.latest_back_gap} "
            f"front_dist={self.latest_front_dist} clearance={self.latest_clearance} | "
            f"cnt_to_i={self.count_to_i} cnt_to_o={self.count_to_o}"
        )

    # ------------------------------------------------------------------ #
    #  Enable / disable service
    # ------------------------------------------------------------------ #
    def on_enable(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = "enabled" if self.enabled else "disabled"
        self.get_logger().info(f"Auto transform -> {response.message}")
        self.count_to_i = 0
        self.count_to_o = 0
        return response

    # ------------------------------------------------------------------ #
    #  Helpers
    # ------------------------------------------------------------------ #
    def can_send(self):
        return (time.time() - self.last_cmd_time) >= self.cooldown

    def send_shape(self, value: int, reason: str):
        if not self.can_send():
            return
        msg = Int32()
        msg.data = int(value)
        self.pub_shape.publish(msg)
        self.current_shape = int(value)
        self.last_cmd_time = time.time()
        self.get_logger().info(f"shape_need={value} ({reason})")

    def set_blocked(self, blocked: bool):
        if blocked != self.blocked:
            self.blocked = blocked
            b = Bool(); b.data = bool(blocked)
            self.pub_blocked.publish(b)
            if blocked:
                self.get_logger().warn("PATH BLOCKED (wall ahead) -> stopping")
            else:
                self.get_logger().info("Path clear -> releasing stop")

    def publish_zero_velocity(self):
        if self.pub_stop_vel:
            self.pub_cmd_vel.publish(Twist())   # all-zero Twist

    def stop_tick(self):
        # While blocked, keep asserting a zero velocity so the robot holds.
        if self.enabled and self.blocked:
            self.publish_zero_velocity()

    # ------------------------------------------------------------------ #
    #  Sensor callbacks
    # ------------------------------------------------------------------ #
    def on_front_dist(self, msg: Float32):
        self.latest_front_dist = float(msg.data)
        # React to a wall ahead immediately (don't wait for the gap callback).
        if self.enabled and self.latest_front_dist is not None:
            blocked = self.latest_front_dist < self.front_stop
            self.set_blocked(blocked)
            if blocked:
                self.count_to_i = 0
                self.count_to_o = 0
                self.publish_zero_velocity()

    def on_back_gap(self, msg: Float32):
        self.latest_back_gap = float(msg.data)

    def on_clearance(self, msg: Float32):
        self.latest_clearance = float(msg.data)

    def on_front_gap(self, msg: Float32):
        self.latest_front_gap = float(msg.data)
        self.decide()

    # ------------------------------------------------------------------ #
    #  Decision logic (driven once per scan by front_gap_min)
    # ------------------------------------------------------------------ #
    def decide(self):
        if not self.enabled:
            return
        if (self.latest_front_gap is None
                or self.latest_back_gap is None
                or self.latest_front_dist is None
                or self.latest_clearance is None):
            return

        # 1) STOP has priority: wall directly ahead ("0 gap").
        if self.latest_front_dist < self.front_stop:
            self.set_blocked(True)
            self.count_to_i = 0
            self.count_to_o = 0
            self.publish_zero_velocity()
            return
        else:
            self.set_blocked(False)

        # 2) LEFT→BACK clearance gate — applies to BOTH O→I and I→O.
        #    The quarter-disc (radius required_clearance, centered 10 cm behind
        #    the lidar, spanning left→back) must be empty before the robot is
        #    allowed to reconfigure in either direction.  inf = empty = clear.
        clearance_ok = self.latest_clearance >= self.required_clr
        if not clearance_ok:
            # Something is in the left-back area: inhibit all transforms and
            # make the robot re-stabilise once it is clear again.
            self.count_to_i = 0
            self.count_to_o = 0
            return

        front_gap = self.latest_front_gap
        back_gap  = self.latest_back_gap

        # O→I : narrow pinch within the front region.
        front_narrow = front_gap < self.threshold

        # I→O : BOTH ends must be open (no narrow pinch) before re-opening.
        #   front open : no narrow gap ahead
        #   back  open : the gap has left the ~60 cm rear window (escaped)
        front_open = front_gap >= self.front_clear_gap
        back_open  = back_gap  >= self.back_clear_gap
        escaped    = front_open and back_open

        # 3) Accumulate stable evidence
        if front_narrow:
            # Narrow gap in front -> want I (and stay I while still inside).
            self.count_to_i += 1
            self.count_to_o = 0
        elif escaped:
            # Front opened AND fully past the gap (back open) -> want O.
            self.count_to_o += 1
            self.count_to_i = 0
        else:
            # Front opened but back NOT yet clear (still escaping) -> hold in I.
            self.count_to_i = 0
            self.count_to_o = 0

        # 4) Trigger O → I
        if self.count_to_i >= self.stable_n:
            if self.current_shape != 1:
                self.send_shape(
                    1,
                    f"front_gap {front_gap:.2f}m < {self.threshold:.2f}m AND "
                    f"left-back clear ({self.latest_clearance:.2f}>="
                    f"{self.required_clr:.2f}) for {self.stable_n} samples (O->I)"
                )
            self.count_to_i = 0

        # 5) Trigger I → O
        if self.count_to_o >= self.stable_n:
            if self.current_shape != 0:
                self.send_shape(
                    0,
                    f"front {front_gap:.2f}m >= {self.front_clear_gap:.2f}m AND "
                    f"back {back_gap:.2f}m >= {self.back_clear_gap:.2f}m AND "
                    f"left-back clear ({self.latest_clearance:.2f}>="
                    f"{self.required_clr:.2f}) for {self.stable_n} samples (I->O)"
                )
            self.count_to_o = 0


def main():
    rclpy.init()
    node = ShapeAutoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()