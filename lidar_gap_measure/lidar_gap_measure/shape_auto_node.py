#!/usr/bin/env python3
import time
from std_srvs.srv import SetBool

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32


class ShapeAutoNode(Node):
    """
    Simplified auto shape transform:
      O→I: front gap (50cm ahead) < 50cm
      I→O: front gap >= 50cm AND back gap (70cm behind) >= 50cm

    Subscribes:
      gap_width      (Float32) - front gap width
      gap_width_back (Float32) - back gap width

    Publishes:
      shape_need (Int32) -> 1 = O→I,  0 = I→O
    """
    def __init__(self):
        super().__init__('shape_auto_node')

        # --- Parameters ---
        self.declare_parameter('gap_topic', 'gap_width')
        self.declare_parameter('back_gap_topic', 'gap_width_back')
        self.declare_parameter('shape_topic', 'shape_need')

        self.declare_parameter('threshold', 0.50)       # 50 cm
        self.declare_parameter('stable_samples', 5)
        self.declare_parameter('cooldown_s', 2.0)

        self.gap_topic      = self.get_parameter('gap_topic').value
        self.back_gap_topic = self.get_parameter('back_gap_topic').value
        self.shape_topic    = self.get_parameter('shape_topic').value

        self.threshold = float(self.get_parameter('threshold').value)
        self.stable_n  = int(self.get_parameter('stable_samples').value)
        self.cooldown  = float(self.get_parameter('cooldown_s').value)

        # --- Internal state ---
        self.current_shape = None   # None=unknown, 0=O, 1=I
        self.last_cmd_time = 0.0
        self.count_to_i = 0
        self.count_to_o = 0

        # Latest sensor values
        self.latest_gap      = None
        self.latest_gap_back = None

        self.enabled = False

        # --- ROS interfaces ---
        self.sub_gap      = self.create_subscription(Float32, self.gap_topic,      self.on_gap, 10)
        self.sub_gap_back = self.create_subscription(Float32, self.back_gap_topic, self.on_gap_back, 10)

        self.pub = self.create_publisher(Int32, self.shape_topic, 10)
        self.srv = self.create_service(SetBool, '/shape_auto/enable', self.on_enable)

        # Debug: print state every 2 seconds
        self.create_timer(2.0, self.debug_log)

        self.get_logger().info(
            f"ShapeAutoNode ready | threshold={self.threshold}m | "
            f"stable_samples={self.stable_n} | cooldown={self.cooldown}s"
        )

    # ------------------------------------------------------------------ #
    #  Debug
    # ------------------------------------------------------------------ #
    def debug_log(self):
        if not self.enabled:
            return
        self.get_logger().info(
            f"[DEBUG] shape={'O' if self.current_shape==0 else 'I' if self.current_shape==1 else '?'} | "
            f"gap_front={self.latest_gap} | gap_back={self.latest_gap_back} | "
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
        self.pub.publish(msg)
        self.current_shape = int(value)
        self.last_cmd_time = time.time()
        self.get_logger().info(f"shape_need={value} ({reason})")

    # ------------------------------------------------------------------ #
    #  Sensor callbacks
    # ------------------------------------------------------------------ #
    def on_gap(self, msg: Float32):
        self.latest_gap = float(msg.data)
        self.decide()

    def on_gap_back(self, msg: Float32):
        self.latest_gap_back = float(msg.data)
        self.decide()

    # ------------------------------------------------------------------ #
    #  Decision logic
    # ------------------------------------------------------------------ #
    def decide(self):
        if not self.enabled:
            return

        if self.latest_gap is None:
            return

        gap = self.latest_gap
        if gap <= 0.0 or gap > 20.0:
            return

        # -------- O → I : front gap is narrow --------
        if gap < self.threshold:
            self.count_to_i += 1
            self.count_to_o = 0

        # -------- I → O : front AND back are both wide --------
        elif gap >= self.threshold:
            back_ok = (
                self.latest_gap_back is not None
                and self.latest_gap_back > 0.0
                and self.latest_gap_back >= self.threshold
            )
            if back_ok:
                self.count_to_o += 1
                self.count_to_i = 0
            else:
                # front is clear but back still narrow or no data
                self.count_to_o = 0
                self.count_to_i = 0

        # -------- Trigger O → I --------
        if self.count_to_i >= self.stable_n:
            if self.current_shape != 1:
                self.send_shape(
                    1,
                    f"front_gap {gap:.2f}m < {self.threshold:.2f}m "
                    f"for {self.stable_n} samples (O->I)"
                )
            self.count_to_i = 0

        # -------- Trigger I → O --------
        if self.count_to_o >= self.stable_n:
            if self.current_shape != 0:
                self.send_shape(
                    0,
                    f"front_gap {gap:.2f}m >= {self.threshold:.2f}m AND "
                    f"back_gap {self.latest_gap_back:.2f}m >= {self.threshold:.2f}m "
                    f"for {self.stable_n} samples (I->O)"
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