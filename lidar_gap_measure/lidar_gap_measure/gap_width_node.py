#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float32MultiArray

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class GapWidthNode(Node):
    """
    - Publishes gap width and a gap marker line at target_x
    - Publishes directional distances (front/left/back/right)
    - Publishes left->back 90deg sector minimum distance (left_back_sector_min)
    - Visualizes directional distances as rays + text in RViz2
    """
    def __init__(self):
        super().__init__('gap_width_node')

        # --- Tunable parameters ---
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('target_x', 0.60)
        self.declare_parameter('x_tol', 0.05)
        self.declare_parameter('range_min_valid', 0.10)
        self.declare_parameter('range_max_valid', 8.0)
        self.declare_parameter('min_points_side', 3)

        # Direction measurement window (+/- degrees around each direction)
        self.declare_parameter('dir_half_angle_deg', 5.0)

        # ✅ NEW: sector clearance settings (LEFT->BACK, 90 degrees)
        self.declare_parameter('sector_start_deg', 90.0)   # left
        self.declare_parameter('sector_end_deg', 180.0)    # back

        scan_topic = self.get_parameter('scan_topic').value
        self.sub = self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)

        self.pub_width = self.create_publisher(Float32, 'gap_width', 10)
        self.pub_marker = self.create_publisher(Marker, 'gap_marker', 10)

        self.pub_dirs = self.create_publisher(Float32MultiArray, 'dir_distances', 10)

        # ✅ NEW: publish min distance in the left->back sector
        self.pub_sector = self.create_publisher(Float32, 'left_back_sector_min', 10)

        # New: visualization for distances
        self.pub_dir_rays = self.create_publisher(MarkerArray, 'dir_rays', 10)
        self.pub_dir_text = self.create_publisher(MarkerArray, 'dir_text', 10)

        self.get_logger().info(
            f"Listening on {scan_topic}. Publishing gap_width, gap_marker, dir_distances, "
            f"left_back_sector_min, dir_rays, dir_text."
        )

    def min_range_in_window(self, angles, ranges, center_rad, half_width_rad):
        diff = (angles - center_rad + math.pi) % (2.0 * math.pi) - math.pi
        mask = np.abs(diff) <= half_width_rad
        if not np.any(mask):
            return float('nan')

        vals = ranges[mask]
        vals = vals[np.isfinite(vals)]
        if vals.size == 0:
            return float('nan')

        return float(np.min(vals))

    # ✅ NEW: minimum range in a sector [start_rad .. end_rad]
    def min_range_in_sector(self, angles, ranges, start_rad, end_rad):
        # normalize to [-pi, pi]
        diff_start = (angles - start_rad + math.pi) % (2.0 * math.pi) - math.pi
        diff_end   = (angles - end_rad   + math.pi) % (2.0 * math.pi) - math.pi

        # sector without wrap (your case 90°..180°)
        if start_rad <= end_rad:
            mask = (angles >= start_rad) & (angles <= end_rad)
        else:
            # wrap-around sector (not used here, but safe)
            mask = (angles >= start_rad) | (angles <= end_rad)

        if not np.any(mask):
            return float('nan')

        vals = ranges[mask]
        vals = vals[np.isfinite(vals)]
        if vals.size == 0:
            return float('nan')

        return float(np.min(vals))

    def make_line_marker(self, header, ns, mid, p_start, p_end, thickness=0.02, rgba=(0.0, 1.0, 1.0, 1.0)):
        mk = Marker()
        mk.header = header
        mk.ns = ns
        mk.id = mid
        mk.type = Marker.LINE_STRIP
        mk.action = Marker.ADD
        mk.pose.orientation.w = 1.0
        mk.scale.x = thickness

        mk.color.r = float(rgba[0])
        mk.color.g = float(rgba[1])
        mk.color.b = float(rgba[2])
        mk.color.a = float(rgba[3])

        mk.lifetime.sec = 0
        mk.lifetime.nanosec = 0

        mk.points = [p_start, p_end]
        return mk

    def make_text_marker(self, header, ns, mid, pos, text, rgba=(1.0, 1.0, 1.0, 1.0), scale_z=0.12):
        mk = Marker()
        mk.header = header
        mk.ns = ns
        mk.id = mid
        mk.type = Marker.TEXT_VIEW_FACING
        mk.action = Marker.ADD

        mk.pose.position = pos
        mk.pose.orientation.w = 1.0

        mk.scale.z = float(scale_z)

        mk.color.r = float(rgba[0])
        mk.color.g = float(rgba[1])
        mk.color.b = float(rgba[2])
        mk.color.a = float(rgba[3])

        mk.text = text

        mk.lifetime.sec = 0
        mk.lifetime.nanosec = 0
        return mk

    def on_scan(self, msg: LaserScan):
        target_x = float(self.get_parameter('target_x').value)
        x_tol = float(self.get_parameter('x_tol').value)
        rmin = float(self.get_parameter('range_min_valid').value)
        rmax = float(self.get_parameter('range_max_valid').value)
        min_points_side = int(self.get_parameter('min_points_side').value)
        dir_half_angle_deg = float(self.get_parameter('dir_half_angle_deg').value)

        # ✅ NEW: sector angles (degrees -> radians)
        sector_start_deg = float(self.get_parameter('sector_start_deg').value)
        sector_end_deg = float(self.get_parameter('sector_end_deg').value)
        sector_start = math.radians(sector_start_deg)
        sector_end = math.radians(sector_end_deg)

        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=np.float32)

        valid = (
            np.isfinite(ranges)
            & (ranges >= max(rmin, msg.range_min))
            & (ranges <= min(rmax, msg.range_max))
        )
        if not np.any(valid):
            return

        ranges = ranges[valid]
        angles = angles[valid]

        # Convert to XY in the scan frame
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        # -------- Directional distances (front/left/back/right) --------
        half_width = math.radians(dir_half_angle_deg)

        front = self.min_range_in_window(angles, ranges, 0.0, half_width)
        left  = self.min_range_in_window(angles, ranges, math.pi / 2.0, half_width)
        back  = self.min_range_in_window(angles, ranges, math.pi, half_width)
        right = self.min_range_in_window(angles, ranges, -math.pi / 2.0, half_width)

        msg_dirs = Float32MultiArray()
        msg_dirs.data = [front, left, back, right]
        self.pub_dirs.publish(msg_dirs)

        # ✅ NEW: Left->Back 90° sector minimum distance
        sector_min = self.min_range_in_sector(angles, ranges, sector_start, sector_end)
        out_sector = Float32()
        out_sector.data = float(sector_min) if math.isfinite(sector_min) else float('nan')
        self.pub_sector.publish(out_sector)

        # Visualize as rays + text
        rays = MarkerArray()
        texts = MarkerArray()

        origin = Point()
        origin.x = 0.0
        origin.y = 0.0
        origin.z = 0.0

        def add_dir(name, mid_base, dist, angle_rad, text_offset=0.10):
            if not math.isfinite(dist):
                return

            end = Point()
            end.x = dist * math.cos(angle_rad)
            end.y = dist * math.sin(angle_rad)
            end.z = 0.0

            rays.markers.append(
                self.make_line_marker(
                    msg.header, "dir_rays", mid_base,
                    origin, end,
                    thickness=0.02,
                    rgba=(0.0, 1.0, 1.0, 1.0)
                )
            )

            tpos = Point()
            tpos.x = end.x + text_offset * math.cos(angle_rad)
            tpos.y = end.y + text_offset * math.sin(angle_rad)
            tpos.z = 0.05

            texts.markers.append(
                self.make_text_marker(
                    msg.header, "dir_text", mid_base,
                    tpos,
                    f"{name}: {dist:.2f} m",
                    rgba=(1.0, 1.0, 1.0, 1.0),
                    scale_z=0.12
                )
            )

        add_dir("Front", 1, front, 0.0)
        add_dir("Left",  2, left,  math.pi / 2.0)
        add_dir("Back",  3, back,  math.pi)
        add_dir("Right", 4, right, -math.pi / 2.0)

        self.pub_dir_rays.publish(rays)
        self.pub_dir_text.publish(texts)

        # -------- Gap width slice measurement --------
        slice_mask = (xs > (target_x - x_tol)) & (xs < (target_x + x_tol))
        if np.count_nonzero(slice_mask) < (2 * min_points_side):
            return

        ys_s = ys[slice_mask]

        left_side = ys_s[ys_s > 0.0]
        right_side = ys_s[ys_s < 0.0]

        if left_side.size < min_points_side or right_side.size < min_points_side:
            return

        y_left = float(np.min(left_side))
        y_right = float(np.max(right_side))

        width = y_left - y_right

        out = Float32()
        out.data = float(width)
        self.pub_width.publish(out)

        # Gap marker
        mk = Marker()
        mk.header = msg.header
        mk.ns = "gap"
        mk.id = 0
        mk.type = Marker.LINE_STRIP
        mk.action = Marker.ADD
        mk.scale.x = 0.02

        mk.color.a = 1.0
        mk.color.r = 1.0
        mk.color.g = 1.0
        mk.color.b = 0.0

        mk.lifetime.sec = 0
        mk.lifetime.nanosec = 0
        mk.pose.orientation.w = 1.0

        p1 = Point()
        p1.x = target_x
        p1.y = y_right
        p1.z = 0.0

        p2 = Point()
        p2.x = target_x
        p2.y = y_left
        p2.z = 0.0

        mk.points = [p1, p2]
        self.pub_marker.publish(mk)


def main():
    rclpy.init()
    node = GapWidthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
