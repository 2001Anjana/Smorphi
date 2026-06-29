#!/usr/bin/env python3
"""
Semantic Mapper Node
====================

Adds semantic detail to the SLAM map while mapping.

Pipeline (single node, no inter-process message contract to maintain):

    camera image  ->  YOLOv8 (ultralytics)  ->  bbox per object
                                                     |
                          bbox horizontal centre -> bearing
                                                     |
                       sample /scan at that bearing -> range  (LiDAR fusion)
                                                     |
                     point in laser frame -> TF -> point in map frame
                                                     |
                   merge into a per-class landmark store (dedup)
                                                     |
        MarkerArray on /semantic_objects  +  persist to ~/smorphi_semantic_map.yaml

This mirrors the existing waypoint_label_publisher pattern (sphere + TEXT_VIEW_FACING
markers in the 'map' frame, YAML persistence) so it drops straight into the same RViz
setup the Smorphi project already uses.

Why fuse with the LiDAR?
    A monocular camera gives bearing only, not distance. The robot already has a 2D
    LiDAR producing /scan. Reading the range at the object's bearing turns a 2D pixel
    box into a real (x, y) point on the same map SLAM is building. If you have an RGB-D
    camera instead, set fusion_method='depth' is NOT implemented here — use 'lidar'
    (recommended for this robot) or 'pose' (crude fallback that just stamps the object
    in front of the robot).
"""

import os
import math
import yaml

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

import tf2_ros
from tf2_geometry_msgs import do_transform_point  # registers PointStamped transform

from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
except Exception as exc:  # pragma: no cover - import guard for clearer error
    YOLO = None
    _YOLO_IMPORT_ERROR = exc


DEFAULT_SAVE_FILE = os.path.expanduser('~/smorphi_semantic_map.yaml')


# A handful of distinct colours so different classes are easy to tell apart in RViz.
# (r, g, b) in 0..1. Cycled if there are more classes than colours.
_PALETTE = [
    (0.96, 0.26, 0.21),  # red
    (0.30, 0.69, 0.31),  # green
    (0.13, 0.59, 0.95),  # blue
    (1.00, 0.76, 0.03),  # amber
    (0.61, 0.15, 0.69),  # purple
    (0.00, 0.74, 0.83),  # cyan
    (1.00, 0.34, 0.13),  # deep orange
    (0.55, 0.76, 0.29),  # lime
]


class Landmark:
    """A single accumulated semantic object on the map."""

    __slots__ = ('cls', 'x', 'y', 'count', 'best_conf')

    def __init__(self, cls, x, y, conf):
        self.cls = cls
        self.x = x
        self.y = y
        self.count = 1
        self.best_conf = conf

    def update(self, x, y, conf):
        # Running average of position -> the estimate tightens as the robot sees the
        # object from more viewpoints, instead of jittering with every frame.
        n = self.count
        self.x = (self.x * n + x) / (n + 1)
        self.y = (self.y * n + y) / (n + 1)
        self.count += 1
        self.best_conf = max(self.best_conf, conf)


class SemanticMapper(Node):

    def __init__(self):
        super().__init__('semantic_mapper')

        # ----- Parameters -------------------------------------------------------
        self.declare_parameter('model_path', os.path.expanduser('~/best.pt'))
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')

        # Camera geometry. Either give the horizontal field of view in degrees, or
        # leave it and the node derives fx from it. cx is taken as image_width / 2.
        self.declare_parameter('horizontal_fov_deg', 70.0)
        # Yaw (radians) from the LiDAR 0-degree direction to the camera's optical
        # axis. 0.0 if the camera looks the same way the LiDAR's 0 ray points.
        self.declare_parameter('camera_yaw_offset', 0.0)

        self.declare_parameter('conf_threshold', 0.5)
        # Only map these class names. Empty list = map every class the model knows.
        self.declare_parameter('class_allowlist', [''])

        # LiDAR fusion settings
        self.declare_parameter('fusion_method', 'lidar')   # 'lidar' or 'pose'
        self.declare_parameter('max_fusion_range', 8.0)    # ignore returns beyond this
        self.declare_parameter('bearing_window_deg', 3.0)  # +/- window to median-filter
        self.declare_parameter('skip_if_no_range', True)   # drop detection if no LiDAR hit
        self.declare_parameter('default_range', 1.0)       # used by 'pose' method / fallback

        # De-duplication: a new detection within merge_radius of an existing landmark
        # of the SAME class updates that landmark instead of creating a new one.
        self.declare_parameter('merge_radius', 0.6)

        # Output / persistence
        self.declare_parameter('marker_topic', '/semantic_objects')
        self.declare_parameter('save_path', DEFAULT_SAVE_FILE)
        self.declare_parameter('save_period', 5.0)
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('load_existing', True)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('annotated_topic', '/yolo/annotated')

        g = self.get_parameter
        self.model_path = g('model_path').value
        self.image_topic = g('image_topic').value
        self.scan_topic = g('scan_topic').value
        self.map_frame = g('map_frame').value
        self.hfov = math.radians(float(g('horizontal_fov_deg').value))
        self.cam_yaw = float(g('camera_yaw_offset').value)
        self.conf_th = float(g('conf_threshold').value)
        allow = [c for c in g('class_allowlist').value if c]
        self.class_allowlist = set(allow)
        self.fusion_method = g('fusion_method').value
        self.max_fusion_range = float(g('max_fusion_range').value)
        self.bearing_window = math.radians(float(g('bearing_window_deg').value))
        self.skip_if_no_range = bool(g('skip_if_no_range').value)
        self.default_range = float(g('default_range').value)
        self.merge_radius = float(g('merge_radius').value)
        self.marker_topic = g('marker_topic').value
        self.save_path = g('save_path').value
        self.publish_annotated = bool(g('publish_annotated').value)
        self.annotated_topic = g('annotated_topic').value

        # ----- Model ------------------------------------------------------------
        if YOLO is None:
            self.get_logger().fatal(
                'Could not import ultralytics. Install it with: '
                'pip3 install ultralytics. Original error: %s' % repr(_YOLO_IMPORT_ERROR))
            raise SystemExit(1)
        if not os.path.exists(self.model_path):
            self.get_logger().fatal('model_path does not exist: %s' % self.model_path)
            raise SystemExit(1)
        self.get_logger().info('Loading YOLO model: %s' % self.model_path)
        self.model = YOLO(self.model_path)
        self.names = self.model.names  # {class_id: class_name}

        # ----- State ------------------------------------------------------------
        self.bridge = CvBridge()
        self.latest_scan = None
        self.landmarks = []          # list[Landmark]
        self._class_color = {}       # class name -> (r,g,b)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----- I/O --------------------------------------------------------------
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_cb, qos_profile_sensor_data)

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(Image, self.annotated_topic, 5)

        if bool(g('load_existing').value):
            self.load_map()

        self.create_timer(float(g('publish_period').value), self.publish_markers)
        self.create_timer(float(g('save_period').value), self.save_map)

        self.get_logger().info('Semantic mapper ready.')
        self.get_logger().info('  image_topic   : %s' % self.image_topic)
        self.get_logger().info('  scan_topic    : %s' % self.scan_topic)
        self.get_logger().info('  fusion_method : %s' % self.fusion_method)
        self.get_logger().info('  markers on    : %s' % self.marker_topic)
        self.get_logger().info('  saving to     : %s' % self.save_path)

    # ---------------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------------
    def scan_cb(self, msg):
        self.latest_scan = msg

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn('cv_bridge conversion failed: %s' % e)
            return

        height, width = frame.shape[:2]
        cx = width / 2.0
        # Focal length in pixels from the horizontal FOV.
        fx = (width / 2.0) / math.tan(self.hfov / 2.0)

        results = self.model(frame, verbose=False)
        if not results:
            return
        res = results[0]

        if self.publish_annotated:
            try:
                annotated = res.plot()  # BGR ndarray with boxes drawn
                out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                out.header = msg.header
                self.annotated_pub.publish(out)
            except Exception as e:
                self.get_logger().debug('annotated publish failed: %s' % e)

        boxes = getattr(res, 'boxes', None)
        if boxes is None or len(boxes) == 0:
            return

        for box in boxes:
            conf = float(box.conf[0])
            if conf < self.conf_th:
                continue
            cls_id = int(box.cls[0])
            cls_name = self.names.get(cls_id, str(cls_id))
            if self.class_allowlist and cls_name not in self.class_allowlist:
                continue

            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            u = (x1 + x2) / 2.0  # horizontal centre pixel

            # Bearing of the object measured from the camera optical axis,
            # positive toward image right.
            alpha = math.atan2((u - cx), fx)
            # Convert to a bearing in the LiDAR/base frame (REP-103: +yaw is CCW/left,
            # image-right is to the robot's right -> negative yaw).
            bearing_laser = self.cam_yaw - alpha

            point_map = self.locate_object(bearing_laser)
            if point_map is None:
                continue
            self.integrate(cls_name, point_map[0], point_map[1], conf)

    # ---------------------------------------------------------------------------
    # Geometry
    # ---------------------------------------------------------------------------
    def locate_object(self, bearing_laser):
        """Return (x, y) of the object in the map frame, or None if it can't be placed."""
        if self.fusion_method == 'lidar':
            rng = self.range_at_bearing(bearing_laser)
            if rng is None:
                if self.skip_if_no_range:
                    return None
                rng = self.default_range
            scan_frame = self.latest_scan.header.frame_id
            px = rng * math.cos(bearing_laser)
            py = rng * math.sin(bearing_laser)
            return self.transform_to_map(px, py, scan_frame)

        # 'pose' method: place the object default_range ahead of the robot, at the
        # detection bearing, using base_link's pose in the map. Crude but dependency-free.
        rng = self.default_range
        px = rng * math.cos(bearing_laser)
        py = rng * math.sin(bearing_laser)
        return self.transform_to_map(px, py, 'base_link')

    def range_at_bearing(self, bearing):
        """Median LiDAR range within +/- bearing_window of `bearing`, or None."""
        scan = self.latest_scan
        if scan is None:
            return None
        n = len(scan.ranges)
        if n == 0:
            return None

        def idx_of(angle):
            return int(round((angle - scan.angle_min) / scan.angle_increment))

        lo = idx_of(bearing - self.bearing_window)
        hi = idx_of(bearing + self.bearing_window)
        if lo > hi:
            lo, hi = hi, lo
        samples = []
        for i in range(lo, hi + 1):
            if i < 0 or i >= n:
                continue
            r = scan.ranges[i]
            if r is None or math.isinf(r) or math.isnan(r):
                continue
            if r <= scan.range_min or r > min(scan.range_max, self.max_fusion_range):
                continue
            samples.append(r)
        if not samples:
            return None
        return float(np.median(samples))

    def transform_to_map(self, x, y, source_frame):
        """Transform a point (x, y, 0) from source_frame into the map frame."""
        pt = PointStamped()
        pt.header.frame_id = source_frame
        pt.header.stamp = rclpy.time.Time().to_msg()  # latest available transform
        pt.point.x = x
        pt.point.y = y
        pt.point.z = 0.0
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, source_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
            out = do_transform_point(pt, tf)
            return (out.point.x, out.point.y)
        except Exception as e:
            self.get_logger().debug('TF %s->%s failed: %s' % (source_frame, self.map_frame, e))
            return None

    # ---------------------------------------------------------------------------
    # Landmark store
    # ---------------------------------------------------------------------------
    def integrate(self, cls_name, x, y, conf):
        """Merge a detection into the nearest same-class landmark, or create one."""
        best = None
        best_d = self.merge_radius
        for lm in self.landmarks:
            if lm.cls != cls_name:
                continue
            d = math.hypot(lm.x - x, lm.y - y)
            if d < best_d:
                best_d = d
                best = lm
        if best is not None:
            best.update(x, y, conf)
        else:
            self.landmarks.append(Landmark(cls_name, x, y, conf))
            self.get_logger().info('New semantic object: %s at (%.2f, %.2f)' % (cls_name, x, y))

    def color_for(self, cls_name):
        if cls_name not in self._class_color:
            self._class_color[cls_name] = _PALETTE[len(self._class_color) % len(_PALETTE)]
        return self._class_color[cls_name]

    # ---------------------------------------------------------------------------
    # Output: RViz markers + YAML persistence
    # ---------------------------------------------------------------------------
    def publish_markers(self):
        if not self.landmarks:
            return
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for idx, lm in enumerate(self.landmarks):
            r, g, b = self.color_for(lm.cls)

            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = now
            sphere.ns = 'semantic_spheres'
            sphere.id = idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = lm.x
            sphere.pose.position.y = lm.y
            sphere.pose.position.z = 0.10
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.20
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = r, g, b, 1.0
            arr.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = now
            text.ns = 'semantic_labels'
            text.id = idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = lm.x
            text.pose.position.y = lm.y
            text.pose.position.z = 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.22
            text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
            text.text = '%s (%.0f%%)' % (lm.cls, lm.best_conf * 100.0)
            arr.markers.append(text)

        self.marker_pub.publish(arr)

    def save_map(self):
        if not self.landmarks:
            return
        data = {}
        counters = {}
        for lm in self.landmarks:
            counters[lm.cls] = counters.get(lm.cls, 0) + 1
            key = '%s_%d' % (lm.cls, counters[lm.cls])
            data[key] = {
                'class': lm.cls,
                'position': {'x': float(lm.x), 'y': float(lm.y), 'z': 0.0},
                'confidence': float(lm.best_conf),
                'observations': int(lm.count),
            }
        try:
            d = os.path.dirname(self.save_path)
            if d:
                os.makedirs(d, exist_ok=True)
            with open(self.save_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
        except Exception as e:
            self.get_logger().warn('Could not save semantic map: %s' % e)

    def load_map(self):
        if not os.path.exists(self.save_path):
            return
        try:
            with open(self.save_path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().warn('Could not load semantic map: %s' % e)
            return
        for _, obj in data.items():
            try:
                lm = Landmark(obj['class'], obj['position']['x'], obj['position']['y'],
                              float(obj.get('confidence', 0.0)))
                lm.count = int(obj.get('observations', 1))
                self.landmarks.append(lm)
            except Exception:
                continue
        self.get_logger().info('Loaded %d existing semantic objects.' % len(self.landmarks))


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_map()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
