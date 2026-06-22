#!/usr/bin/env python3
"""
path_comparison_plotter.py
===========================
ROS2 node that subscribes to:
  - /plan          (nav_msgs/Path)     — Nav2 planned global path
  - /amcl_pose     (geometry_msgs/PoseWithCovarianceStamped)  — actual localized pose
  - /odom          (nav_msgs/Odometry) — wheel odometry (fallback if AMCL not used)

It builds up the actual path by accumulating poses over time,
then displays a live matplotlib plot comparing planned vs actual path.

Usage (while Nav2 is running):
  ros2 run smorphi_ros2_launchers path_comparison_plotter

Save the graph image at any point by pressing 's' in the plot window.
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

import matplotlib
matplotlib.use('TkAgg')           # use TkAgg so it works on desktop; change to 'Qt5Agg' if needed
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import math
import threading
import os
import datetime


class PathComparisonPlotter(Node):
    def __init__(self):
        super().__init__('path_comparison_plotter')

        # ── Parameters ──────────────────────────────────────────────────────────
        self.declare_parameter('pose_source', 'amcl')   # 'amcl' | 'odom'
        self.declare_parameter('min_dist_m', 0.05)      # min distance between recorded poses
        self.declare_parameter('save_dir', os.path.expanduser('~/path_logs'))
        self.declare_parameter('plot_title', 'Smorphi — Planned vs Actual Path')

        self.pose_source  = self.get_parameter('pose_source').value
        self.min_dist     = self.get_parameter('min_dist_m').value
        self.save_dir     = self.get_parameter('save_dir').value
        self.plot_title   = self.get_parameter('plot_title').value

        os.makedirs(self.save_dir, exist_ok=True)

        # ── Data storage ─────────────────────────────────────────────────────────
        self._lock            = threading.Lock()
        self.planned_x: list  = []
        self.planned_y: list  = []
        self.actual_x:  list  = []
        self.actual_y:  list  = []
        self._last_x:   float = None
        self._last_y:   float = None

        # ── ROS2 subscriptions ───────────────────────────────────────────────────
        self.plan_sub = self.create_subscription(
            Path,
            '/plan',
            self._plan_cb,
            10
        )

        if self.pose_source == 'amcl':
            self.pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose',
                self._amcl_cb,
                10
            )
            self.get_logger().info('Tracking actual pose from: /amcl_pose')
        else:
            self.pose_sub = self.create_subscription(
                Odometry,
                '/odom',
                self._odom_cb,
                10
            )
            self.get_logger().info('Tracking actual pose from: /odom')

        # ── Matplotlib setup ─────────────────────────────────────────────────────
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(9, 7))
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self._setup_plot()

        # Update plot at 2 Hz
        self.timer = self.create_timer(0.5, self._update_plot)

        self.get_logger().info(
            f'PathComparisonPlotter started. Press "s" in the plot window to save.')

    # ─── Callbacks ──────────────────────────────────────────────────────────────

    def _plan_cb(self, msg: Path):
        """Store the planned path whenever a new global plan is received."""
        xs = [pose.pose.position.x for pose in msg.poses]
        ys = [pose.pose.position.y for pose in msg.poses]
        with self._lock:
            self.planned_x = xs
            self.planned_y = ys
        self.get_logger().info(f'New plan received: {len(xs)} waypoints')

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._record_actual(x, y)

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._record_actual(x, y)

    def _record_actual(self, x: float, y: float):
        """Append pose only if the robot has moved more than min_dist."""
        if self._last_x is not None:
            dist = math.hypot(x - self._last_x, y - self._last_y)
            if dist < self.min_dist:
                return
        with self._lock:
            self.actual_x.append(x)
            self.actual_y.append(y)
        self._last_x = x
        self._last_y = y

    # ─── Plot ────────────────────────────────────────────────────────────────────

    def _setup_plot(self):
        self.ax.set_title(self.plot_title, fontsize=13, fontweight='bold')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_aspect('equal', 'box')
        self.ax.grid(True, linestyle='--', alpha=0.5)
        planned_patch = mpatches.Patch(color='royalblue',  label='Planned Path (/plan)')
        actual_patch  = mpatches.Patch(color='crimson',    label='Actual Path (pose)')
        self.ax.legend(handles=[planned_patch, actual_patch], loc='upper left')

    def _update_plot(self):
        with self._lock:
            px = list(self.planned_x)
            py = list(self.planned_y)
            ax = list(self.actual_x)
            ay = list(self.actual_y)

        self.ax.cla()
        self._setup_plot()

        if px:
            self.ax.plot(px, py,
                         color='royalblue', linewidth=2.0,
                         label='Planned Path', zorder=2)
            # Mark start & goal
            self.ax.scatter([px[0]],  [py[0]],  color='green',  s=80, zorder=5, label='Start')
            self.ax.scatter([px[-1]], [py[-1]], color='orange', s=80, marker='*',
                            zorder=5, label='Goal')

        if ax:
            self.ax.plot(ax, ay,
                         color='crimson', linewidth=1.5, linestyle='--',
                         label='Actual Path', zorder=3)
            # Mark robot current position
            self.ax.scatter([ax[-1]], [ay[-1]],
                            color='crimson', s=100, marker='D', zorder=6)

        # Show cross-track error stats if both paths have data
        if len(px) > 1 and len(ax) > 1:
            errors = self._cross_track_errors(
                np.column_stack((px, py)),
                np.column_stack((ax, ay))
            )
            mean_e = np.mean(errors)
            max_e  = np.max(errors)
            self.ax.set_title(
                f'{self.plot_title}\n'
                f'Mean cross-track error: {mean_e:.3f} m   |   Max: {max_e:.3f} m',
                fontsize=11, fontweight='bold'
            )

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    # ─── Utility ─────────────────────────────────────────────────────────────────

    def _cross_track_errors(self, planned: np.ndarray, actual: np.ndarray) -> np.ndarray:
        """For each actual pose, find distance to nearest point on the planned path."""
        errors = []
        for pt in actual:
            # Vectorised distance to all planned points
            dists = np.linalg.norm(planned - pt, axis=1)
            errors.append(dists.min())
        return np.array(errors)

    def _on_key(self, event):
        if event.key == 's':
            self._save_plot()

    def _save_plot(self):
        ts   = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.save_dir, f'path_comparison_{ts}.png')
        self.fig.savefig(path, dpi=150, bbox_inches='tight')
        self.get_logger().info(f'Plot saved → {path}')

    def save_csv(self):
        """Call on shutdown to dump raw data."""
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        with self._lock:
            px, py = list(self.planned_x), list(self.planned_y)
            ax, ay = list(self.actual_x),  list(self.actual_y)

        planned_path = os.path.join(self.save_dir, f'planned_{ts}.csv')
        actual_path  = os.path.join(self.save_dir, f'actual_{ts}.csv')

        np.savetxt(planned_path, np.column_stack((px, py)) if px else np.empty((0,2)),
                   delimiter=',', header='x,y', comments='')
        np.savetxt(actual_path,  np.column_stack((ax, ay)) if ax else np.empty((0,2)),
                   delimiter=',', header='x,y', comments='')

        self.get_logger().info(f'CSVs saved → {self.save_dir}')


def main(args=None):
    rclpy.init(args=args)
    node = PathComparisonPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
