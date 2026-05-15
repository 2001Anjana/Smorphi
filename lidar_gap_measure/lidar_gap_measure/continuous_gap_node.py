#!/usr/bin/env python3
"""
continuous_gap_node
───────────────────
Sweeps gap-width slices from x_start (+0.20 m, ahead) to x_end (−0.60 m,
behind) in x_step increments and publishes the **minimum** gap width found
anywhere in that corridor.

Publishes
─────────
  gap_width_min   (Float32)        – narrowest gap across all slices
  gap_markers     (MarkerArray)    – coloured slice lines for RViz
  gap_text_min    (Marker)         – text label at the narrowest gap
  dir_distances   (Float32MultiArray) – front/left/back/right distances
  left_back_sector_min (Float32)   – min range in the left→back sector
  dir_rays / dir_text (MarkerArray)– directional distance visualisation
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class ContinuousGapNode(Node):

    def __init__(self):
        super().__init__('continuous_gap_node')

        # ── scan parameters ──
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('range_min_valid', 0.10)
        self.declare_parameter('range_max_valid', 8.0)
        self.declare_parameter('min_points_side', 3)

        # ── sweep range ──
        self.declare_parameter('x_start', 0.20)   # front-most slice (m)
        self.declare_parameter('x_end',  -0.60)    # rear-most slice  (m)
        self.declare_parameter('x_step',  0.05)    # step size        (m)
        self.declare_parameter('x_tol',   0.05)    # half-width of each slice

        # ── directional distance window ──
        self.declare_parameter('dir_half_angle_deg', 5.0)

        # ── sector clearance (left→back, 90°) ──
        self.declare_parameter('sector_start_deg', 90.0)
        self.declare_parameter('sector_end_deg',  180.0)

        # ── subscriptions ──
        scan_topic = self.get_parameter('scan_topic').value
        self.sub = self.create_subscription(LaserScan, scan_topic,
                                            self.on_scan, 10)

        # ── publishers ──
        self.pub_gap_min   = self.create_publisher(Float32,      'gap_width_min',  10)
        self.pub_markers   = self.create_publisher(MarkerArray,  'gap_markers',    10)
        self.pub_gap_text  = self.create_publisher(Marker,       'gap_text_min',   10)
        self.pub_dirs      = self.create_publisher(Float32MultiArray, 'dir_distances', 10)
        self.pub_sector    = self.create_publisher(Float32,      'left_back_sector_min', 10)
        self.pub_dir_rays  = self.create_publisher(MarkerArray,  'dir_rays', 10)
        self.pub_dir_text  = self.create_publisher(MarkerArray,  'dir_text', 10)

        self.get_logger().info(
            f"ContinuousGapNode ready | sweep {self.get_parameter('x_start').value}"
            f" -> {self.get_parameter('x_end').value} m, "
            f"step {self.get_parameter('x_step').value} m"
        )

    # ─────────────────────── helpers ───────────────────────

    @staticmethod
    def _min_range_in_window(angles, ranges, center_rad, half_width_rad):
        diff = (angles - center_rad + math.pi) % (2.0 * math.pi) - math.pi
        mask = np.abs(diff) <= half_width_rad
        if not np.any(mask):
            return float('nan')
        vals = ranges[mask]
        vals = vals[np.isfinite(vals)]
        return float(np.min(vals)) if vals.size else float('nan')

    @staticmethod
    def _min_range_in_sector(angles, ranges, start_rad, end_rad):
        if start_rad <= end_rad:
            mask = (angles >= start_rad) & (angles <= end_rad)
        else:
            mask = (angles >= start_rad) | (angles <= end_rad)
        if not np.any(mask):
            return float('nan')
        vals = ranges[mask]
        vals = vals[np.isfinite(vals)]
        return float(np.min(vals)) if vals.size else float('nan')

    def _make_line_marker(self, header, ns, mid, p_start, p_end,
                          thickness=0.02, rgba=(0., 1., 1., 1.)):
        mk = Marker()
        mk.header = header
        mk.ns     = ns
        mk.id     = mid
        mk.type   = Marker.LINE_STRIP
        mk.action = Marker.ADD
        mk.pose.orientation.w = 1.0
        mk.scale.x = thickness
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = (
            float(c) for c in rgba)
        mk.lifetime.sec = 0
        mk.lifetime.nanosec = 300_000_000   # 0.3 s so stale lines vanish
        mk.points = [p_start, p_end]
        return mk

    def _make_text_marker(self, header, ns, mid, pos, text,
                          rgba=(1., 1., 1., 1.), scale_z=0.12):
        mk = Marker()
        mk.header = header
        mk.ns     = ns
        mk.id     = mid
        mk.type   = Marker.TEXT_VIEW_FACING
        mk.action = Marker.ADD
        mk.pose.position = pos
        mk.pose.orientation.w = 1.0
        mk.scale.z = float(scale_z)
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = (
            float(c) for c in rgba)
        mk.text = text
        mk.lifetime.sec = 0
        mk.lifetime.nanosec = 300_000_000
        return mk

    # ───────────── gap width for a single X slice ─────────────

    def _gap_at_x(self, xs, ys, target_x, x_tol, min_pts):
        """Return (gap_width, y_left, y_right) or None if insufficient data."""
        mask = (xs > (target_x - x_tol)) & (xs < (target_x + x_tol))
        if np.count_nonzero(mask) < 2 * min_pts:
            return None

        ys_s = ys[mask]
        left_side  = ys_s[ys_s > 0.0]
        right_side = ys_s[ys_s < 0.0]

        if left_side.size < min_pts or right_side.size < min_pts:
            return None

        y_left  = float(np.min(left_side))
        y_right = float(np.max(right_side))
        return (y_left - y_right, y_left, y_right)

    # ─────────────────────── scan callback ───────────────────────

    def on_scan(self, msg: LaserScan):
        x_start = float(self.get_parameter('x_start').value)
        x_end   = float(self.get_parameter('x_end').value)
        x_step  = float(self.get_parameter('x_step').value)
        x_tol   = float(self.get_parameter('x_tol').value)
        rmin    = float(self.get_parameter('range_min_valid').value)
        rmax    = float(self.get_parameter('range_max_valid').value)
        min_pts = int(self.get_parameter('min_points_side').value)
        dir_half = math.radians(
            float(self.get_parameter('dir_half_angle_deg').value))
        sec_s = math.radians(float(self.get_parameter('sector_start_deg').value))
        sec_e = math.radians(float(self.get_parameter('sector_end_deg').value))

        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=np.float32)

        valid = (np.isfinite(ranges)
                 & (ranges >= max(rmin, msg.range_min))
                 & (ranges <= min(rmax, msg.range_max)))
        if not np.any(valid):
            return

        ranges = ranges[valid]
        angles = angles[valid]
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        # ──── directional distances ────
        front = self._min_range_in_window(angles, ranges, 0.0, dir_half)
        left  = self._min_range_in_window(angles, ranges, math.pi / 2, dir_half)
        back  = self._min_range_in_window(angles, ranges, math.pi, dir_half)
        right = self._min_range_in_window(angles, ranges, -math.pi / 2, dir_half)

        d = Float32MultiArray()
        d.data = [front, left, back, right]
        self.pub_dirs.publish(d)

        # ──── sector min ────
        sec_min = self._min_range_in_sector(angles, ranges, sec_s, sec_e)
        sm = Float32()
        sm.data = float(sec_min) if math.isfinite(sec_min) else float('nan')
        self.pub_sector.publish(sm)

        # ──── directional rays + text visualisation ────
        self._publish_dir_vis(msg.header, front, left, back, right)

        # ──── sweep gap slices ────
        # Build the list of target_x values from x_start down to x_end
        if x_start > x_end:
            targets = np.arange(x_start, x_end - 1e-9, -abs(x_step))
        else:
            targets = np.arange(x_start, x_end + 1e-9, abs(x_step))

        min_gap   = float('inf')
        min_gap_x = 0.0
        min_yl    = 0.0
        min_yr    = 0.0

        markers = MarkerArray()
        mid = 0

        for tx in targets:
            result = self._gap_at_x(xs, ys, float(tx), x_tol, min_pts)
            if result is None:
                continue

            gap_w, yl, yr = result

            # colour: green (wide) -> red (narrow), threshold at 0.6 m
            ratio = max(0.0, min(1.0, gap_w / 0.60))
            r_col = 1.0 - ratio
            g_col = ratio

            p1 = Point(x=float(tx), y=yr, z=0.0)
            p2 = Point(x=float(tx), y=yl, z=0.0)
            markers.markers.append(
                self._make_line_marker(
                    msg.header, 'gap_sweep', mid, p1, p2,
                    thickness=0.015,
                    rgba=(r_col, g_col, 0.0, 0.8)))
            mid += 1

            if gap_w < min_gap:
                min_gap   = gap_w
                min_gap_x = float(tx)
                min_yl    = yl
                min_yr    = yr

        # publish markers even if empty (clears old markers)
        self.pub_markers.publish(markers)

        if math.isinf(min_gap):
            return   # no valid slices at all

        # ──── publish minimum gap width ────
        out = Float32()
        out.data = float(min_gap)
        self.pub_gap_min.publish(out)

        # ──── text label at narrowest gap ────
        txt = self._make_text_marker(
            msg.header, 'gap_text_min', 0,
            Point(x=min_gap_x, y=(min_yl + min_yr) / 2.0, z=0.15),
            f"Min gap: {min_gap:.2f} m @ x={min_gap_x:.2f}",
            rgba=(1.0, 0.3, 0.3, 1.0),
            scale_z=0.13)
        self.pub_gap_text.publish(txt)

    # ──── directional distance visualisation ────

    def _publish_dir_vis(self, header, front, left, back, right):
        rays  = MarkerArray()
        texts = MarkerArray()
        origin = Point(x=0.0, y=0.0, z=0.0)

        def add(name, mid, dist, angle_rad, off=0.10):
            if not math.isfinite(dist):
                return
            end = Point(x=dist * math.cos(angle_rad),
                        y=dist * math.sin(angle_rad), z=0.0)
            rays.markers.append(
                self._make_line_marker(header, 'dir_rays', mid,
                                       origin, end, 0.02, (0., 1., 1., 1.)))
            tp = Point(x=end.x + off * math.cos(angle_rad),
                       y=end.y + off * math.sin(angle_rad), z=0.05)
            texts.markers.append(
                self._make_text_marker(header, 'dir_text', mid, tp,
                                       f"{name}: {dist:.2f} m"))

        add('Front', 1, front, 0.0)
        add('Left',  2, left,  math.pi / 2)
        add('Back',  3, back,  math.pi)
        add('Right', 4, right, -math.pi / 2)

        self.pub_dir_rays.publish(rays)
        self.pub_dir_text.publish(texts)


def main():
    rclpy.init()
    node = ContinuousGapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
