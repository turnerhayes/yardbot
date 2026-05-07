#!/usr/bin/env python3
"""
yardbot_bringup/fence_boundary_node.py

Two-mode fence boundary node:

MAPPING MODE (default on first run, no state file exists):
  - Continuously collects tag poses from the TF tree in map frame
  - Accumulates multiple observations per tag and averages them for stability
  - Call the ~/commit_fence service to cluster all observed tags into fence
    lines, save to state file, and switch to navigation mode

NAVIGATION MODE (state file exists):
  - Loads saved fence lines from state file on startup
  - Publishes /fence_layer OccupancyGrid immediately
  - Does not update fence lines during operation
  - Call ~/commit_fence again to re-map (e.g. after repositioning tags)

Clustering logic:
  - Tags whose inferred fence lines are approximately parallel (within
    angle_cluster_tol radians) AND close together (within
    perp_cluster_tol metres perpendicular distance) are grouped into one
    fence line
  - The best-fit line through all tags in a cluster is used

The fence layer is published as a nav_msgs/OccupancyGrid on /fence_layer
with TRANSIENT_LOCAL QoS, suitable for use as a Nav2 static costmap layer.

Parameters:
  tag_family         (str,   default "36h11")
  map_frame          (str,   default "map")
  update_rate        (float, default 1.0)      Hz, mapping mode only
  state_file         (str,   default "~/.ros/yardbot_fence.json")
  wall_cost          (int,   default 100)      0-100
  half_width_cells   (int,   default 2)        wall thickness (cells each side)
  angle_cluster_tol  (float, default 0.175)    ~10 degrees in radians
  perp_cluster_tol   (float, default 0.3)      metres

Services:
  ~/commit_fence  (std_srvs/srv/Trigger)
    Cluster current tag observations into fence lines, save, and publish.
    Works in both modes — call again to re-map after repositioning tags.

  ~/clear_fence   (std_srvs/srv/Trigger)
    Delete state file and clear all fence lines. Returns to mapping mode.
"""

import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple, cast

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_srvs.srv import Trigger
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException # pyright: ignore[reportAttributeAccessIssue]

# ── Geometry helpers ──────────────────────────────────────────────────────────

def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    """Wrap angle to [-pi/2, pi/2] — lines have no direction."""
    a = a % math.pi
    if a > math.pi / 2:
        a -= math.pi
    return a


def perpendicular_distance(px: float, py: float,
                           lx: float, ly: float, yaw: float) -> float:
    """Signed perpendicular distance from point (px,py) to line through
    (lx,ly) at angle yaw."""
    dx = math.cos(yaw)
    dy = math.sin(yaw)
    return -(px - lx) * dy + (py - ly) * dx


def fit_line(points: List[Tuple[float, float]]) -> Tuple[float, float, float]:
    """
    Fit a line to a list of (x, y) points using PCA (first principal component).
    Returns (cx, cy, yaw) where (cx, cy) is the centroid and yaw is the
    line direction in [-pi/2, pi/2].
    """
    n = len(points)
    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n
    if n == 1:
        return cx, cy, 0.0
    sxx = sum((p[0] - cx) ** 2 for p in points)
    syy = sum((p[1] - cy) ** 2 for p in points)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in points)
    yaw = 0.5 * math.atan2(2 * sxy, sxx - syy)
    return cx, cy, normalize_angle(yaw)


# ── Data classes ──────────────────────────────────────────────────────────────

class TagObservation:
    """Running-averaged pose of a single tag in map frame."""
    def __init__(self, tag_id: str, x: float, y: float, yaw: float):
        self.tag_id = tag_id
        self.x = x
        self.y = y
        self.yaw = normalize_angle(yaw)
        self._count = 1

    def update(self, x: float, y: float, yaw: float):
        self._count += 1
        a = 1.0 / self._count
        self.x = self.x * (1 - a) + x * a
        self.y = self.y * (1 - a) + y * a
        # Circular mean for angle
        self.yaw = math.atan2(
            math.sin(self.yaw) * (1 - a) + math.sin(yaw) * a,
            math.cos(self.yaw) * (1 - a) + math.cos(yaw) * a,
        )
        self.yaw = normalize_angle(self.yaw)


class FenceLine:
    """A committed fence line stored in the state file."""
    def __init__(self, fence_id: str, x: float, y: float, yaw: float,
                 tag_ids: List[str]):
        self.fence_id = fence_id
        self.x = x
        self.y = y
        self.yaw = yaw
        self.tag_ids = tag_ids

    def to_dict(self) -> dict:
        return {
            "fence_id": self.fence_id,
            "x": self.x, "y": self.y, "yaw": self.yaw,
            "tag_ids": self.tag_ids,
        }

    @staticmethod
    def from_dict(d: dict) -> "FenceLine":
        return FenceLine(
            d["fence_id"], d["x"], d["y"], d["yaw"], d.get("tag_ids", [])
        )


# ── Node ──────────────────────────────────────────────────────────────────────

class FenceBoundaryNode(Node):

    def __init__(self):
        super().__init__("fence_boundary")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("tag_family",        "36h11")
        self.declare_parameter("map_frame",         "map")
        self.declare_parameter("update_rate",       1.0)
        self.declare_parameter("state_file",        "~/.ros/yardbot_fence.json")
        self.declare_parameter("wall_cost",         100)
        self.declare_parameter("half_width_cells",  2)
        self.declare_parameter("angle_cluster_tol", 0.175)
        self.declare_parameter("perp_cluster_tol",  0.3)

        self.tag_family       = cast(str, self.get_parameter("tag_family").value)
        self.map_frame        = cast(str, self.get_parameter("map_frame").value)
        self.update_rate      = cast(float, self.get_parameter("update_rate").value)
        self.state_file       = Path(
            cast(str, self.get_parameter("state_file").value)).expanduser()
        self.wall_cost        = cast(int, self.get_parameter("wall_cost").value)
        self.half_width_cells = cast(int, self.get_parameter("half_width_cells").value)
        self.angle_tol        = cast(float, self.get_parameter("angle_cluster_tol").value)
        self.perp_tol         = cast(float, self.get_parameter("perp_cluster_tol").value)

        # ── TF ────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Publisher ─────────────────────────────────────────────────────
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.fence_pub = self.create_publisher(
            OccupancyGrid, "/fence_layer", latched_qos)

        # ── Map subscription ──────────────────────────────────────────────
        self.latest_map: Optional[OccupancyGrid] = None
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, 10)

        # ── Services ──────────────────────────────────────────────────────
        self.create_service(Trigger, "~/commit_fence", self._commit_cb)
        self.create_service(Trigger, "~/clear_fence",  self._clear_cb)

        # ── State ─────────────────────────────────────────────────────────
        self.tag_observations: Dict[str, TagObservation] = {}
        self.fence_lines: List[FenceLine] = []
        self.mapping_mode = True

        if self.state_file.exists():
            self._load_state()
        else:
            self.get_logger().info(
                "No state file found — starting in MAPPING MODE. "
                "Drive around to expose all tags, then call ~/commit_fence."
            )

        # ── Timer ─────────────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / self.update_rate, self._mapping_update)

    # ── Map callback ──────────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid):
        prev = self.latest_map
        self.latest_map = msg
        if prev is None or (
            prev.info.width      != msg.info.width  or
            prev.info.height     != msg.info.height or
            prev.info.resolution != msg.info.resolution
        ):
            self.get_logger().info(
                f"Map received: {msg.info.width}x{msg.info.height} "
                f"@ {msg.info.resolution:.3f} m/cell"
            )
            if not self.mapping_mode and self.fence_lines:
                self._publish_fence()

    # ── Mapping mode timer ────────────────────────────────────────────────

    def _mapping_update(self):
        if not self.mapping_mode:
            return
        all_frames = self.tf_buffer.all_frames_as_string()
        for line in all_frames.splitlines():
            parts = line.split()
            if len(parts) < 2:
                continue
            frame_id = parts[1]
            if not frame_id.startswith(f"tag{self.tag_family}:"):
                continue
            obs = self._lookup_tag(frame_id)
            if obs is None:
                continue
            existing = self.tag_observations.get(frame_id)
            if existing is None:
                self.tag_observations[frame_id] = obs
                self.get_logger().info(
                    f"[MAPPING] First observation of {frame_id}: "
                    f"({obs.x:.3f}, {obs.y:.3f}) "
                    f"yaw={math.degrees(obs.yaw):.1f}°"
                )
            else:
                existing.update(obs.x, obs.y, obs.yaw)

    def _lookup_tag(self, tag_frame: str) -> Optional[TagObservation]:
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, tag_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (LookupException,
                ConnectivityException,
                ExtrapolationException):
            return None
        tr = t.transform.translation
        ro = t.transform.rotation
        yaw = yaw_from_quaternion(ro.x, ro.y, ro.z, ro.w)
        return TagObservation(tag_frame, tr.x, tr.y, yaw)

    # ── Clustering ────────────────────────────────────────────────────────

    def _cluster_tags(self) -> List[FenceLine]:
        """
        Group tag observations into fence lines.

        Two tags belong to the same fence if:
          1. Their yaw angles differ by less than angle_tol
          2. The perpendicular distance between their lines is < perp_tol

        Within each cluster, fit a best-fit line through all tag positions.
        """
        observations = list(self.tag_observations.values())
        if not observations:
            return []

        clusters: List[List[TagObservation]] = []
        for obs in observations:
            placed = False
            for cluster in clusters:
                rep = cluster[0]
                angle_diff = abs(normalize_angle(obs.yaw - rep.yaw))
                if angle_diff > self.angle_tol:
                    continue
                perp = abs(perpendicular_distance(
                    obs.x, obs.y, rep.x, rep.y, rep.yaw))
                if perp > self.perp_tol:
                    continue
                cluster.append(obs)
                placed = True
                break
            if not placed:
                clusters.append([obs])

        fence_lines = []
        for i, cluster in enumerate(clusters):
            points = [(o.x, o.y) for o in cluster]
            cx, cy, yaw = fit_line(points)
            tag_ids = [o.tag_id for o in cluster]
            fence_lines.append(FenceLine(
                fence_id=f"fence_{i}",
                x=cx, y=cy, yaw=yaw,
                tag_ids=tag_ids,
            ))
            self.get_logger().info(
                f"Cluster fence_{i}: tags={tag_ids} → "
                f"({cx:.3f}, {cy:.3f}) yaw={math.degrees(yaw):.1f}°"
            )
        return fence_lines

    # ── Services ──────────────────────────────────────────────────────────

    def _commit_cb(self, request, response):
        if not self.tag_observations:
            response.success = False
            response.message = (
                "No tag observations collected yet. "
                "Drive around to expose tags first."
            )
            return response

        self.fence_lines = self._cluster_tags()
        self._save_state()
        self.mapping_mode = False
        self._publish_fence()

        response.success = True
        response.message = (
            f"Committed {len(self.fence_lines)} fence line(s) from "
            f"{len(self.tag_observations)} tag(s). Switched to NAVIGATION MODE."
        )
        self.get_logger().info(response.message)
        return response

    def _clear_cb(self, request, response):
        self.fence_lines = []
        self.tag_observations = {}
        self.mapping_mode = True
        if self.state_file.exists():
            self.state_file.unlink()
        response.success = True
        response.message = "Fence cleared. Back in MAPPING MODE."
        self.get_logger().info(response.message)
        return response

    # ── Grid rendering ────────────────────────────────────────────────────

    def _publish_fence(self):
        if self.latest_map is None:
            self.get_logger().warn("Cannot publish fence: no map received yet.")
            return
        if not self.fence_lines:
            return

        info     = self.latest_map.info
        width    = info.width
        height   = info.height
        res      = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        data = [0] * (width * height)
        for fence in self.fence_lines:
            self._draw_fence_line(
                data, width, height, res, origin_x, origin_y, fence)

        msg = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info = MapMetaData(
            map_load_time=self.get_clock().now().to_msg(),
            resolution=res,
            width=width,
            height=height,
            origin=info.origin,
        )
        msg.data = data
        self.fence_pub.publish(msg)
        self.get_logger().info(
            f"Published fence layer: {len(self.fence_lines)} line(s)")

    def _draw_fence_line(self, data, width, height, res,
                         origin_x, origin_y, fence: FenceLine):
        dx = math.cos(fence.yaw)
        dy = math.sin(fence.yaw)
        cx = (fence.x - origin_x) / res
        cy = (fence.y - origin_y) / res
        max_steps = int(math.hypot(width, height)) + 1
        perp_x = -dy
        perp_y =  dx
        for offset in range(-self.half_width_cells, self.half_width_cells + 1):
            sx = cx + offset * perp_x
            sy = cy + offset * perp_y
            for direction in (1, -1):
                for step in range(max_steps):
                    gx = int(round(sx + direction * step * dx))
                    gy = int(round(sy + direction * step * dy))
                    if 0 <= gx < width and 0 <= gy < height:
                        data[gy * width + gx] = self.wall_cost
                    else:
                        break

    # ── Persistence ───────────────────────────────────────────────────────

    def _save_state(self):
        try:
            self.state_file.parent.mkdir(parents=True, exist_ok=True)
            with open(self.state_file, "w") as f:
                json.dump([fl.to_dict() for fl in self.fence_lines], f, indent=2)
            self.get_logger().info(f"Saved fence state to {self.state_file}")
        except OSError as e:
            self.get_logger().error(f"Failed to save fence state: {e}")

    def _load_state(self):
        try:
            with open(self.state_file) as f:
                raw = json.load(f)
            self.fence_lines = [FenceLine.from_dict(d) for d in raw]
            self.mapping_mode = False
            self.get_logger().info(
                f"Loaded {len(self.fence_lines)} fence line(s) from "
                f"{self.state_file} — NAVIGATION MODE."
            )
        except (OSError, KeyError, json.JSONDecodeError) as e:
            self.get_logger().error(
                f"Failed to load fence state: {e} — starting in MAPPING MODE.")
            self.fence_lines = []
            self.mapping_mode = True


def main(args=None):
    rclpy.init(args=args)
    node = FenceBoundaryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
