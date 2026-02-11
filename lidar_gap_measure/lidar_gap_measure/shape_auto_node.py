#!/usr/bin/env python3
import time
from std_srvs.srv import SetBool

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Float32MultiArray


class ShapeAutoNode(Node):
    """
    Auto shape transform based on:
      - gap_width
      - clearance in 90deg sector from LEFT->BACK (quarter circle area)

    Subscribes:
      gap_width (Float32)
      dir_distances (Float32MultiArray)  data=[front,left,back,right]  (kept)
      left_back_sector_min (Float32)     # NEW: min range in 90° sector [left..back]

    Publishes:
      shape_need (Int32) -> 1: O->I, 0: I->O
    """
    def __init__(self):
        super().__init__('shape_auto_node')

        # Params (existing)
        self.declare_parameter('gap_topic', 'gap_width')
        self.declare_parameter('dirs_topic', 'dir_distances')
        self.declare_parameter('shape_topic', 'shape_need')

        self.declare_parameter('threshold_i', 0.35)
        self.declare_parameter('threshold_o', 0.40)

        # ✅ NEW param for the sector topic
        self.declare_parameter('sector_topic', 'left_back_sector_min')

        # Keep your required radius
        self.declare_parameter('required_radius', 0.40)

        self.declare_parameter('stable_samples', 5)
        self.declare_parameter('cooldown_s', 2.0)

        self.gap_topic = self.get_parameter('gap_topic').value
        self.dirs_topic = self.get_parameter('dirs_topic').value
        self.shape_topic = self.get_parameter('shape_topic').value
        self.sector_topic = self.get_parameter('sector_topic').value

        self.th_i = float(self.get_parameter('threshold_i').value)
        self.th_o = float(self.get_parameter('threshold_o').value)
        self.req_r = float(self.get_parameter('required_radius').value)

        self.stable_n = int(self.get_parameter('stable_samples').value)
        self.cooldown = float(self.get_parameter('cooldown_s').value)

        # Internal state (existing)
        self.current_shape = None
        self.last_cmd_time = 0.0
        self.count_below = 0
        self.count_above = 0

        # Latest sensor values (existing + new)
        self.latest_gap = None
        self.latest_left = None
        self.latest_back = None
        self.latest_sector_min = None  # ✅ NEW

        self.enabled = False

        # ROS interfaces (existing)
        self.sub_gap = self.create_subscription(Float32, self.gap_topic, self.on_gap, 10)
        self.sub_dirs = self.create_subscription(Float32MultiArray, self.dirs_topic, self.on_dirs, 10)
        self.sub_sector = self.create_subscription(Float32, self.sector_topic, self.on_sector, 10)  # ✅ NEW

        self.pub = self.create_publisher(Int32, self.shape_topic, 10)
        self.srv = self.create_service(SetBool, '/shape_auto/enable', self.on_enable)

    def on_enable(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = "enabled" if self.enabled else "disabled"
        self.get_logger().info(f"Auto transform -> {response.message}")
        self.count_below = 0
        self.count_above = 0
        return response

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

    # --- Callbacks (unchanged structure) ---
    def on_gap(self, msg: Float32):
        self.latest_gap = float(msg.data)
        self.on_decide()

    def on_dirs(self, msg: Float32MultiArray):
        if msg.data is None or len(msg.data) < 4:
            return
        left = float(msg.data[1])
        back = float(msg.data[2])

        if left <= 0.0 or left > 20.0:
            left = None
        if back <= 0.0 or back > 20.0:
            back = None

        self.latest_left = left
        self.latest_back = back
        self.on_decide()

    def on_sector(self, msg: Float32):
        d = float(msg.data)
        if d <= 0.0 or d > 20.0:
            d = None
        self.latest_sector_min = d
        self.on_decide()

    def on_decide(self):
        if not self.enabled:
            return

        # Need gap + sector clearance
        if self.latest_gap is None or self.latest_sector_min is None:
            return

        gap = float(self.latest_gap)
        sector_min = float(self.latest_sector_min)

        if gap <= 0.0 or gap > 20.0:
            return

        # ✅ NEW: quarter-circle (90° sector) clearance condition
        sector_ok = (sector_min >= self.req_r)

        if (gap < self.th_i) and sector_ok:
            self.count_below += 1
            self.count_above = 0
        elif gap > self.th_o:
            self.count_above += 1
            self.count_below = 0
        else:
            self.count_below = 0
            self.count_above = 0

        if self.count_below >= self.stable_n:
            if self.current_shape != 1:
                self.send_shape(
                    1,
                    f"gap {gap:.2f}m < {self.th_i:.2f}m AND sector_min {sector_min:.2f}>= {self.req_r:.2f} "
                    f"for {self.stable_n} samples (O->I)"
                )
            self.count_below = 0

        if self.count_above >= self.stable_n:
            if self.current_shape != 0:
                self.send_shape(
                    0,
                    f"gap {gap:.2f}m > {self.th_o:.2f}m for {self.stable_n} samples (I->O)"
                )
            self.count_above = 0


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
