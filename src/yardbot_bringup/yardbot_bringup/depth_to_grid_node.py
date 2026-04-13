#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped

from cv_bridge import CvBridge

import tf2_ros
from tf2_ros import TransformException

from message_filters import Subscriber, ApproximateTimeSynchronizer


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Quaternion to 3x3 rotation matrix."""
    # normalized quaternion assumed
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx + zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx + yy)],
    ], dtype=np.float32)


def apply_tf(T: TransformStamped, pts: np.ndarray) -> np.ndarray:
    """
    Apply geometry_msgs/TransformStamped to Nx3 points.
    pts in source frame -> returned points in target frame.
    """
    t = T.transform.translation
    q = T.transform.rotation
    R = quat_to_rot(q.x, q.y, q.z, q.w)
    trans = np.array([t.x, t.y, t.z], dtype=np.float32)
    return (pts @ R.T) + trans  # (N,3)


class DepthToGridNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_to_grid")

        # Parameters
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/depth/camera_info")
        self.declare_parameter("target_frame", "base_link")

        self.declare_parameter("resolution", 0.05)          # meters/cell
        self.declare_parameter("width_m", 10.0)             # grid width in meters
        self.declare_parameter("height_m", 10.0)            # grid height in meters
        self.declare_parameter("origin_x_m", -5.0)          # origin in target_frame
        self.declare_parameter("origin_y_m", -5.0)

        self.declare_parameter("min_range_m", 0.2)
        self.declare_parameter("max_range_m", 6.0)

        # Height filter: keep obstacle points within [z_min, z_max] in target_frame
        self.declare_parameter("z_min_m", 0.05)
        self.declare_parameter("z_max_m", 0.40)

        # Sampling stride for speed
        self.declare_parameter("stride", 4)

        # Occupancy values
        self.declare_parameter("occ_value", 100)  # occupied cell value
        self.declare_parameter("free_value", 0)   # (not used in this simple version)
        self.declare_parameter("unknown_value", -1)

        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.grid_pub = self.create_publisher(OccupancyGrid, "occupancy", 10)

        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("camera_info_topic").value

        self.depth_sub = Subscriber(self, Image, depth_topic)
        self.info_sub = Subscriber(self, CameraInfo, info_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.info_sub],
            queue_size=5,
            slop=0.05
        )
        self.sync.registerCallback(self.cb)

        self.get_logger().info("DepthToGridNode started.")

    def cb(self, depth_msg: Image, info_msg: CameraInfo) -> None:
        target_frame = self.get_parameter("target_frame").value

        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                depth_msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")
            return

        # Intrinsics
        fx = info_msg.k[0]
        fy = info_msg.k[4]
        cx = info_msg.k[2]
        cy = info_msg.k[5]

        # Depth image to numpy
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth = np.array(depth)

        # Convert depth units to meters
        # RealSense typically publishes 16UC1 in millimeters
        if depth_msg.encoding == "16UC1":
            depth_m = depth.astype(np.float32) * 0.001
        else:
            # assume already meters for 32FC1
            depth_m = depth.astype(np.float32)

        stride = int(self.get_parameter("stride").value)
        depth_s = depth_m[::stride, ::stride]
        h, w = depth_s.shape

        # Pixel grids in the subsampled image
        us = (np.arange(w, dtype=np.float32) * stride)
        vs = (np.arange(h, dtype=np.float32) * stride)
        uu, vv = np.meshgrid(us, vs)

        z = depth_s
        min_r = float(self.get_parameter("min_range_m").value)
        max_r = float(self.get_parameter("max_range_m").value)

        valid = np.isfinite(z) & (z > min_r) & (z < max_r)
        if not np.any(valid):
            return

        z_valid = z[valid]
        u_valid = uu[valid]
        v_valid = vv[valid]

        # Backproject to camera frame
        x = (u_valid - cx) * z_valid / fx
        y = (v_valid - cy) * z_valid / fy
        pts_cam = np.stack([x, y, z_valid], axis=1).astype(np.float32)

        # Transform to target frame
        pts = apply_tf(tf, pts_cam)

        # Height filtering in target frame
        zmin = float(self.get_parameter("z_min_m").value)
        zmax = float(self.get_parameter("z_max_m").value)
        keep = (pts[:, 2] >= zmin) & (pts[:, 2] <= zmax)
        pts = pts[keep]
        if pts.shape[0] == 0:
            return

        # Create grid
        res = float(self.get_parameter("resolution").value)
        width_m = float(self.get_parameter("width_m").value)
        height_m = float(self.get_parameter("height_m").value)
        origin_x = float(self.get_parameter("origin_x_m").value)
        origin_y = float(self.get_parameter("origin_y_m").value)

        grid_w = int(math.ceil(width_m / res))
        grid_h = int(math.ceil(height_m / res))

        unknown = int(self.get_parameter("unknown_value").value)
        occ_val = int(self.get_parameter("occ_value").value)

        grid = np.full((grid_h, grid_w), unknown, dtype=np.int8)

        # Project points to grid indices (x forward, y left in typical base_link; adjust as needed)
        gx = np.floor((pts[:, 0] - origin_x) / res).astype(np.int32)
        gy = np.floor((pts[:, 1] - origin_y) / res).astype(np.int32)

        inb = (gx >= 0) & (gx < grid_w) & (gy >= 0) & (gy < grid_h)
        gx = gx[inb]
        gy = gy[inb]

        # Mark occupied
        grid[gy, gx] = occ_val

        # Publish OccupancyGrid
        msg = OccupancyGrid()
        msg.header.stamp = depth_msg.header.stamp
        msg.header.frame_id = target_frame
        msg.info.resolution = res
        msg.info.width = grid_w
        msg.info.height = grid_h
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.reshape(-1).tolist()

        self.grid_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = DepthToGridNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()