#!/usr/bin/env python3
"""
continuous_gap_node
───────────────────
Sweeps gap-width slices from x_start (FRONT, +x) to x_end (BACK, −x) in x_step
increments. Publishes the minimum gap of the whole corridor (kept for
visualisation / backward compatibility) AND, separately, the measurements the
shape-transform logic actually uses:

  • front_gap_min : narrowest gap in the FRONT region only (x in
                    [front_x_min, front_x_max], default 0.0 .. 0.40 m).
                    This is what decides O→I, so a wall BEHIND the robot can
                    no longer trigger a transform.

  • back_gap_min  : narrowest gap in the BACK region only (x in
                    [back_x_min, back_x_max], default −0.60 .. −0.05 m).
                    This is what confirms the robot has FULLY escaped the gap
                    (I→O).  It is a GAP WIDTH between the left/right walls, NOT
                    a straight-back distance.  This is the important fix: the
                    old `back_distance` cone shot straight down the corridor and
                    read large clearance even while the robot's rear was still
                    flanked by the gap walls, which made the robot re-open (I→O)
                    *before* it had physically cleared the gap.  By measuring the
                    actual gap width behind the robot, the gap is still seen as
                    "narrow" until it leaves the rear window (≈60 cm behind),
                    so the robot stays in I until it has truly escaped.
                    99.0 = no walls on both sides in the rear window -> escaped.

  • front_distance: lidar-to-front nearest-obstacle distance (straight ahead).
                    Used to STOP when a wall blocks the path.
  • back_distance : lidar-to-back nearest-obstacle distance (straight behind).
                    Kept for visualisation / compatibility ONLY — no longer used
                    by the escape (I→O) logic.

Publishes
─────────
  gap_width_min        (Float32)            – narrowest gap across all slices
  front_gap_min        (Float32)            – narrowest gap in the FRONT region
                                              (99.0 = no walls / open ahead)
  back_gap_min         (Float32)            – narrowest gap in the BACK region
                                              (99.0 = no walls / escaped behind)
  front_distance       (Float32)            – lidar→front distance (inf = clear)
  back_distance        (Float32)            – lidar→back  distance (inf = clear)
  gap_markers          (MarkerArray)        – coloured slice lines for RViz
  gap_text_min         (Marker)             – text label at the narrowest gap
  dir_distances        (Float32MultiArray)  – front/left/back/right distances
  left_back_sector_min (Float32)            – min range in the left→back sector
  left_back_clearance_min (Float32)         – nearest obstacle inside the offset
                                              left→back quarter-disc (inf = clear)
  clearance_markers    (MarkerArray)        – filled quarter-disc, GREEN=clear /
                                              RED=blocked (topic: clearance_markers)
  dir_rays / dir_text  (MarkerArray)        – directional distance visualisation
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
        # x_start must reach the front-most slice we want to inspect (0.40 m),
        # x_end must reach the back-most slice we want to inspect (−0.60 m).
        self.declare_parameter('x_start', 0.40)    # front-most slice (m)
        self.declare_parameter('x_end',  -0.60)    # rear-most slice  (m)
        self.declare_parameter('x_step',  0.05)    # step size        (m)
        self.declare_parameter('x_tol',   0.05)    # half-width of each slice

        # ── front region used for the O→I decision ──
        # Extended from 0.20 m to 0.40 m so a narrow gap is detected earlier
        # (up to 40 cm ahead) giving the robot time to transform before reaching
        # the pinch.
        self.declare_parameter('front_x_min', 0.00)   # 0 cm  (at the lidar)
        self.declare_parameter('front_x_max', 0.40)   # 40 cm ahead

        # ── back region used for the I→O (escape) decision ──
        # The gap is considered "still there" while any rear slice in this band
        # shows two close walls.  When the gap leaves this band (i.e. it is more
        # than ~60 cm behind the lidar) the rear reads "open" and the robot may
        # re-open to O.
        self.declare_parameter('back_x_min', -0.60)   # 60 cm behind
        self.declare_parameter('back_x_max', -0.05)   # just behind the lidar

        # ── directional distance window ──
        self.declare_parameter('dir_half_angle_deg', 5.0)

        # ── sector clearance (left→back, 90°) ──
        self.declare_parameter('sector_start_deg', 90.0)
        self.declare_parameter('sector_end_deg',  180.0)

        # ── left→back clearance quarter-disc ──
        # The robot may only reconfigure (O↔I) when this region is empty.
        # Center sits `clearance_center_back` metres BEHIND the lidar (−x),
        # and the quarter disc of radius `clearance_radius` spans
        # LEFT (+y) → BACK (−x):  points with dx <= 0 (behind C) and dy >= 0.
        self.declare_parameter('clearance_center_back', 0.10)  # 10 cm behind lidar
        self.declare_parameter('clearance_radius', 0.40)       # 40 cm radius

        # ── subscriptions ──
        scan_topic = self.get_parameter('scan_topic').value
        self.sub = self.create_subscription(LaserScan, scan_topic,
                                            self.on_scan, 10)

        # ── publishers ──
        self.pub_gap_min    = self.create_publisher(Float32,      'gap_width_min',  10)
        self.pub_front_gap  = self.create_publisher(Float32,      'front_gap_min',  10)
        self.pub_back_gap   = self.create_publisher(Float32,      'back_gap_min',   10)
        self.pub_front_dist = self.create_publisher(Float32,      'front_distance', 10)
        self.pub_back_dist  = self.create_publisher(Float32,      'back_distance',  10)
        self.pub_markers    = self.create_publisher(MarkerArray,  'gap_markers',    10)
        self.pub_gap_text   = self.create_publisher(Marker,       'gap_text_min',   10)
        self.pub_dirs       = self.create_publisher(Float32MultiArray, 'dir_distances', 10)
        self.pub_sector     = self.create_publisher(Float32,      'left_back_sector_min', 10)
        self.pub_clearance  = self.create_publisher(Float32,      'left_back_clearance_min', 10)
        self.pub_clear_vis  = self.create_publisher(MarkerArray,  'clearance_markers', 10)
        self.pub_dir_rays   = self.create_publisher(MarkerArray,  'dir_rays', 10)
        self.pub_dir_text   = self.create_publisher(MarkerArray,  'dir_text', 10)

        self.get_logger().info(
            f"ContinuousGapNode ready | sweep {self.get_parameter('x_start').value}"
            f" -> {self.get_parameter('x_end').value} m, "
            f"step {self.get_parameter('x_step').value} m | "
            f"front region [{self.get_parameter('front_x_min').value}, "
            f"{self.get_parameter('front_x_max').value}] m | "
            f"back region [{self.get_parameter('back_x_min').value}, "
            f"{self.get_parameter('back_x_max').value}] m"
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

    @staticmethod
    def _left_back_clearance(xs, ys, center_x):
        """
        Nearest obstacle distance from the offset center C=(center_x, 0) to any
        lidar point inside the LEFT→BACK quarter-disc.

        Quadrant relative to C:
            behind C : dx = (x - center_x) <= 0
            left  C  : dy =  y             >= 0

        Returns the minimum distance from C to a point in that wedge, or +inf
        when the wedge is empty (nothing there -> fully clear).  The caller
        compares this against the disc radius to decide clear vs blocked.
        """
        dx = xs - center_x          # center_x is negative (behind the lidar)
        dy = ys
        in_quadrant = (dx <= 0.0) & (dy >= 0.0)
        if not np.any(in_quadrant):
            return float('inf')
        dist = np.sqrt(dx[in_quadrant] ** 2 + dy[in_quadrant] ** 2)
        dist = dist[np.isfinite(dist)]
        return float(np.min(dist)) if dist.size else float('inf')

    def _make_clearance_markers(self, header, center_x, radius, clear):
        """
        Filled LEFT→BACK quarter-disc for RViz.
          GREEN translucent  = clear  (robot may transform)
          RED   translucent  = blocked (transform inhibited)
        Returns a MarkerArray with a filled sector (id 0) + bright outline (id 1).
        """
        arr = MarkerArray()
        if clear:
            fill = (0.00, 0.39, 0.00, 0.30)      # dark green, translucent fill
            edge = (0.00, 0.50, 0.00, 0.95)      # dark green, bright outline
        else:
            fill = (0.85, 0.10, 0.10, 0.35)
            edge = (1.00, 0.15, 0.15, 0.95)

        # Arc points sweep 90° (LEFT, +y) -> 180° (BACK, −x) around C.
        n = 24
        arc = []
        for i in range(n + 1):
            a = math.pi / 2.0 + (math.pi / 2.0) * (i / n)
            arc.append(Point(x=center_x + radius * math.cos(a),
                             y=radius * math.sin(a), z=0.0))
        center_pt = Point(x=center_x, y=0.0, z=0.0)

        # ── filled sector (triangle fan) ──
        fillm = Marker()
        fillm.header = header
        fillm.ns     = 'clearance_fill'
        fillm.id     = 0
        fillm.type   = Marker.TRIANGLE_LIST
        fillm.action = Marker.ADD
        fillm.pose.orientation.w = 1.0
        fillm.scale.x = 1.0
        fillm.scale.y = 1.0
        fillm.scale.z = 1.0
        fillm.color.r, fillm.color.g, fillm.color.b, fillm.color.a = (
            float(c) for c in fill)
        fillm.lifetime.sec = 0
        fillm.lifetime.nanosec = 300_000_000
        tris = []
        for i in range(n):
            tris.extend([center_pt, arc[i], arc[i + 1]])
        fillm.points = tris
        arr.markers.append(fillm)

        # ── bright outline ──
        edgem = Marker()
        edgem.header = header
        edgem.ns     = 'clearance_edge'
        edgem.id     = 1
        edgem.type   = Marker.LINE_STRIP
        edgem.action = Marker.ADD
        edgem.pose.orientation.w = 1.0
        edgem.scale.x = 0.02
        edgem.color.r, edgem.color.g, edgem.color.b, edgem.color.a = (
            float(c) for c in edge)
        edgem.lifetime.sec = 0
        edgem.lifetime.nanosec = 300_000_000
        edgem.points = [center_pt] + arc + [center_pt]
        arr.markers.append(edgem)
        return arr

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
        """Return (gap_width, y_left, y_right) or None if insufficient data.

        A slice only yields a gap when there are walls on BOTH sides
        (left: y>0, right: y<0).  If either side is missing the slice is
        treated as "open" (None) — i.e. no narrow pinch at that x.
        """
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
        fx_min  = float(self.get_parameter('front_x_min').value)
        fx_max  = float(self.get_parameter('front_x_max').value)
        bx_min  = float(self.get_parameter('back_x_min').value)
        bx_max  = float(self.get_parameter('back_x_max').value)
        dir_half = math.radians(
            float(self.get_parameter('dir_half_angle_deg').value))
        sec_s = math.radians(float(self.get_parameter('sector_start_deg').value))
        sec_e = math.radians(float(self.get_parameter('sector_end_deg').value))
        clr_back = float(self.get_parameter('clearance_center_back').value)
        clr_r    = float(self.get_parameter('clearance_radius').value)
        clr_cx   = -abs(clr_back)   # center sits BEHIND the lidar (−x)

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

        # ──── front / back straight distances (front used for STOP) ────
        # inf means "nothing detected in that direction" -> treated as clear.
        fd = Float32()
        fd.data = float(front) if math.isfinite(front) else float('inf')
        self.pub_front_dist.publish(fd)

        bd = Float32()
        bd.data = float(back) if math.isfinite(back) else float('inf')
        self.pub_back_dist.publish(bd)   # compat / viz only

        # ──── sector min ────
        sec_min = self._min_range_in_sector(angles, ranges, sec_s, sec_e)
        sm = Float32()
        sm.data = float(sec_min) if math.isfinite(sec_min) else float('nan')
        self.pub_sector.publish(sm)

        # ──── left→back clearance quarter-disc ────
        # nearest obstacle from C inside the wedge; inf = empty = clear.
        clr_min = self._left_back_clearance(xs, ys, clr_cx)
        clear = (clr_min >= clr_r)
        cm = Float32()
        cm.data = float(clr_min)
        self.pub_clearance.publish(cm)
        self.pub_clear_vis.publish(
            self._make_clearance_markers(msg.header, clr_cx, clr_r, clear))

        # ──── directional rays + text visualisation ────
        self._publish_dir_vis(msg.header, front, left, back, right)

        # ──── sweep gap slices ────
        if x_start > x_end:
            targets = np.arange(x_start, x_end - 1e-9, -abs(x_step))
        else:
            targets = np.arange(x_start, x_end + 1e-9, abs(x_step))

        min_gap   = float('inf')      # whole-corridor minimum (viz/compat)
        min_gap_x = 0.0
        min_yl    = 0.0
        min_yr    = 0.0
        front_gap = float('inf')      # minimum within the FRONT region only
        back_gap  = float('inf')      # minimum within the BACK region only

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

            # front-region minimum (only slices inside [fx_min, fx_max])
            if (fx_min - 1e-9) <= float(tx) <= (fx_max + 1e-9):
                if gap_w < front_gap:
                    front_gap = gap_w

            # back-region minimum (only slices inside [bx_min, bx_max])
            if (bx_min - 1e-9) <= float(tx) <= (bx_max + 1e-9):
                if gap_w < back_gap:
                    back_gap = gap_w

        # publish markers even if empty (clears old markers)
        self.pub_markers.publish(markers)

        # ──── publish FRONT-region gap (99.0 = open / no walls ahead) ────
        fg = Float32()
        fg.data = float(front_gap) if math.isfinite(front_gap) else 99.0
        self.pub_front_gap.publish(fg)

        # ──── publish BACK-region gap (99.0 = open / escaped behind) ────
        bg = Float32()
        bg.data = float(back_gap) if math.isfinite(back_gap) else 99.0
        self.pub_back_gap.publish(bg)

        if math.isinf(min_gap):
            return   # no valid slices anywhere

        # ──── publish whole-corridor minimum gap (compat / viz) ────
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