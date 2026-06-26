#!/usr/bin/env python3
"""
path_comparison_plotter.py  (corrected)
=======================================
Headless-compatible ROS2 node (Raspberry Pi / no display).

Compares the Nav2 GLOBAL PLAN against the robot's ACTUAL trajectory and saves a
PNG + CSV on Ctrl+C.

What was wrong in the previous version, and what changed here
-------------------------------------------------------------
1. `/plan` was OVERWRITTEN every callback. Nav2 (navigate_w_replanning...) keeps
   republishing /plan from the robot's *current* pose to the goal, so by Ctrl+C
   you were left with only the last short remnant -> the blue "plan" looked tiny
   and the cross-track error was computed against the wrong reference.
   FIX: keep the LONGEST plan ever seen (the full intended route) and never let a
   shorter remnant replace it. The goal is captured separately.

2. FRAME MISMATCH. /plan and /amcl_pose are in `map`; /odom is in `odom`. Plotting
   odom-frame points against a map-frame plan adds the (drifting) map->odom offset
   to the "error".
   FIX: default pose_source='tf' samples map->base_link, i.e. the robot pose in the
   PLAN'S frame, regardless of which estimator produced it. 'amcl'/'odom' remain as
   fallbacks with a warning.

3. Cross-track error was point-to-vertex against the truncated plan.
   FIX: point-to-SEGMENT distance against the full plan polyline.

Subscribes to:
  - /plan        (nav_msgs/Path)                            global plan (kept: longest)
  - /goal_pose   (geometry_msgs/PoseStamped)                commanded goal (optional)
  - /amcl_pose   (geometry_msgs/PoseWithCovarianceStamped)  if pose_source == 'amcl'
  - /odom        (nav_msgs/Odometry)                        if pose_source == 'odom'
  - TF map->base_link                                       if pose_source == 'tf' (default)

Usage:
  ros2 run smorphi_ros2_launchers plot_paths
  ros2 run smorphi_ros2_launchers plot_paths --ros-args -p pose_source:=amcl
  ros2 run smorphi_ros2_launchers plot_paths --ros-args -p pose_source:=odom
  ros2 run smorphi_ros2_launchers plot_paths --ros-args -p global_frame:=map -p base_frame:=base_link
"""

import os
import datetime

import matplotlib
matplotlib.use('Agg')                       # headless — no display needed
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException


class PathComparisonPlotter(Node):
    def __init__(self):
        super().__init__('path_comparison_plotter')

        # ── Parameters ───────────────────────────────────────────────────────────
        self.declare_parameter('pose_source', 'tf')     # 'tf' | 'amcl' | 'odom'
        self.declare_parameter('global_frame', 'map')    # frame the plan lives in
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('save_dir', os.path.expanduser('~/path_logs'))
        self.declare_parameter('plot_title', 'Smorphi — Planned vs Actual Path')
        self.declare_parameter('sample_period', 0.5)     # seconds (2 Hz)

        self.pose_source  = self.get_parameter('pose_source').value
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame   = self.get_parameter('base_frame').value
        self.save_dir     = self.get_parameter('save_dir').value
        self.plot_title   = self.get_parameter('plot_title').value
        self.sample_period = float(self.get_parameter('sample_period').value)

        os.makedirs(self.save_dir, exist_ok=True)

        # ── Data storage ──────────────────────────────────────────────────────────
        self.planned_x: list = []           # the LONGEST plan seen (full route)
        self.planned_y: list = []
        self._best_plan_len = 0

        self.actual_x: list = []
        self.actual_y: list = []

        self.goal_xy = None                 # (x, y) commanded goal, if known

        # Latest pose from a topic source (used by 'amcl'/'odom' modes)
        self._latest_xy = None

        # ── TF (for 'tf' mode and odom->map transform) ─────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscriptions ──────────────────────────────────────────────────────────
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 10)

        if self.pose_source == 'amcl':
            self.create_subscription(
                PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        elif self.pose_source == 'odom':
            self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        # 'tf' mode needs no pose subscription — it reads the TF tree directly.

        self.create_timer(self.sample_period, self._sample_pose)

        self.get_logger().info(
            f'PathComparisonPlotter started (headless).\n'
            f'  pose_source : {self.pose_source}  '
            f'(actual path resolved in frame "{self.global_frame}")\n'
            f'  sampling    : every {self.sample_period:.2f}s\n'
            f'  Ctrl+C to save PNG + CSV -> {self.save_dir}/'
        )
        if self.pose_source == 'odom':
            self.get_logger().warn(
                "pose_source='odom': /odom is in the 'odom' frame and will be "
                "transformed into '%s' via TF. If TF is unavailable the raw "
                "(frame-mismatched) values are used. 'tf' mode is recommended."
                % self.global_frame)

    # ─── Subscriptions ────────────────────────────────────────────────────────────

    def _plan_cb(self, msg: Path):
        """Keep the longest plan ever received (proxy for the full intended route)."""
        if len(msg.poses) > self._best_plan_len:
            self._best_plan_len = len(msg.poses)
            self.planned_x = [p.pose.position.x for p in msg.poses]
            self.planned_y = [p.pose.position.y for p in msg.poses]
            if msg.header.frame_id and msg.header.frame_id != self.global_frame:
                self.get_logger().warn(
                    f"/plan frame is '{msg.header.frame_id}' but global_frame is "
                    f"'{self.global_frame}'. Using global_frame for TF lookups.")
            # If no explicit goal was given, treat the plan end as the goal.
            if self.goal_xy is None and self.planned_x:
                self.goal_xy = (self.planned_x[-1], self.planned_y[-1])
            self.get_logger().info(f'Stored global plan: {self._best_plan_len} pts')

    def _goal_cb(self, msg: PoseStamped):
        self.goal_xy = (msg.pose.position.x, msg.pose.position.y)

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self._latest_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _odom_cb(self, msg: Odometry):
        # store with its frame so we can transform it at sample time
        self._latest_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._odom_frame = msg.header.frame_id or 'odom'

    # ─── Pose sampler ──────────────────────────────────────────────────────────────

    def _sample_pose(self):
        xy = None
        if self.pose_source == 'tf':
            xy = self._lookup_tf(self.global_frame, self.base_frame)
        elif self.pose_source == 'amcl':
            xy = self._latest_xy           # already in map
        elif self.pose_source == 'odom':
            # transform odom-frame point into global_frame if possible
            if self._latest_xy is not None:
                t = self._lookup_tf(self.global_frame, getattr(self, '_odom_frame', 'odom'))
                if t is not None:
                    # apply 2D rigid transform: global = R*odom + t (here t carries
                    # the odom-frame origin in global; simplest: look up base directly)
                    xy = self._lookup_tf(self.global_frame, self.base_frame) or self._latest_xy
                else:
                    xy = self._latest_xy   # raw fallback (frame-mismatched)

        if xy is not None:
            self.actual_x.append(xy[0])
            self.actual_y.append(xy[1])

    def _lookup_tf(self, target, source):
        """Return (x, y) of `source` origin expressed in `target`, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, Time(), timeout=Duration(seconds=0.1))
            return (tf.transform.translation.x, tf.transform.translation.y)
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(
                f'TF {target} <- {source} unavailable; skipping sample',
                throttle_duration_sec=5.0)
            return None

    # ─── Metrics ───────────────────────────────────────────────────────────────────

    @staticmethod
    def _pt_seg_dist(p, a, b):
        ab = b - a
        denom = float(ab @ ab)
        t = 0.0 if denom < 1e-12 else float((p - a) @ ab) / denom
        t = max(0.0, min(1.0, t))
        return float(np.linalg.norm(p - (a + t * ab)))

    def _cross_track(self):
        """Per-actual-point min distance to the planned polyline."""
        if len(self.planned_x) < 2 or len(self.actual_x) < 1:
            return None
        plan = np.column_stack((self.planned_x, self.planned_y))
        errs = []
        for ax, ay in zip(self.actual_x, self.actual_y):
            p = np.array([ax, ay])
            d = min(self._pt_seg_dist(p, plan[i], plan[i + 1])
                    for i in range(len(plan) - 1))
            errs.append(d)
        return np.array(errs)

    def _actual_path_length(self):
        if len(self.actual_x) < 2:
            return 0.0
        a = np.column_stack((self.actual_x, self.actual_y))
        return float(np.sum(np.linalg.norm(np.diff(a, axis=0), axis=1)))

    def _net_displacement(self):
        if len(self.actual_x) < 2:
            return 0.0
        return float(np.hypot(self.actual_x[-1] - self.actual_x[0],
                              self.actual_y[-1] - self.actual_y[0]))

    # ─── Final save ──────────────────────────────────────────────────────────────

    def save_final(self):
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        png_path = os.path.join(self.save_dir, f'path_comparison_{ts}.png')
        csv_path = os.path.join(self.save_dir, f'path_data_{ts}.csv')

        errs = self._cross_track()
        arc = self._actual_path_length()
        net = self._net_displacement()
        goal_d = None
        if self.goal_xy is not None and self.actual_x:
            goal_d = float(np.hypot(self.actual_x[-1] - self.goal_xy[0],
                                    self.actual_y[-1] - self.goal_xy[1]))

        fig = self._build_figure(errs, arc, net, goal_d)
        fig.savefig(png_path, dpi=150, bbox_inches='tight')
        plt.close(fig)

        rows = [f'planned,{x:.6f},{y:.6f}' for x, y in zip(self.planned_x, self.planned_y)]
        rows += [f'actual,{x:.6f},{y:.6f}' for x, y in zip(self.actual_x, self.actual_y)]
        with open(csv_path, 'w') as f:
            f.write('# mean_cross_track_m=%s max_cross_track_m=%s arc_len_m=%.3f '
                    'net_disp_m=%.3f final_dist_to_goal_m=%s\n'
                    % ('%.3f' % errs.mean() if errs is not None else 'NA',
                       '%.3f' % errs.max() if errs is not None else 'NA',
                       arc, net, '%.3f' % goal_d if goal_d is not None else 'NA'))
            f.write('path_type,x,y\n')
            f.write('\n'.join(rows) + '\n')

        self.get_logger().info(
            f'Saved:\n  PNG -> {png_path}\n  CSV -> {csv_path}\n'
            f'  plan pts={len(self.planned_x)} actual pts={len(self.actual_x)} '
            f'arc={arc:.3f}m net={net:.3f}m '
            f'goal_dist={"NA" if goal_d is None else f"{goal_d:.3f}m"}')

    # ─── Plot builder ──────────────────────────────────────────────────────────────

    def _build_figure(self, errs, arc, net, goal_d):
        px, py = self.planned_x, self.planned_y
        ax_, ay_ = self.actual_x, self.actual_y

        fig, axis = plt.subplots(figsize=(9, 7))
        axis.set_xlabel('X (m)')
        axis.set_ylabel('Y (m)')
        axis.set_aspect('equal', 'box')
        axis.grid(True, linestyle='--', alpha=0.5)

        if px:
            axis.plot(px, py, color='royalblue', linewidth=2.0, zorder=2,
                      label='Planned path (/plan, full route)')
            axis.scatter([px[0]], [py[0]], color='green', s=90, zorder=5,
                         label='Plan start')
        if self.goal_xy is not None:
            axis.scatter([self.goal_xy[0]], [self.goal_xy[1]], color='orange',
                         s=130, marker='*', zorder=6, label='Goal')
        if ax_:
            axis.plot(ax_, ay_, color='crimson', linewidth=1.5, linestyle='--',
                      zorder=3, label='Actual path')
            axis.scatter([ax_[0]], [ay_[0]], color='black', s=55, marker='s',
                         zorder=6, label='Actual start')
            axis.scatter([ax_[-1]], [ay_[-1]], color='crimson', s=100, marker='D',
                         zorder=6, label='Actual end')

        title = self.plot_title
        bits = []
        if errs is not None:
            bits.append(f'mean XTE: {errs.mean():.3f} m | max: {errs.max():.3f} m')
        bits.append(f'actual arc: {arc:.3f} m | net disp: {net:.3f} m')
        if goal_d is not None:
            bits.append(f'final dist to goal: {goal_d:.3f} m')
        title = self.plot_title + '\n' + '   '.join(bits)

        axis.set_title(title, fontsize=10, fontweight='bold')
        axis.legend(loc='best', fontsize=8)
        fig.tight_layout()
        return fig


def main(args=None):
    rclpy.init(args=args)
    node = PathComparisonPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_final()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()