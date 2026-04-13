#!/usr/bin/env python3
"""
apriltag_overlay_node.py
Subscribes to:
  - /camera/camera/color/image_raw        (sensor_msgs/Image)
  - /camera/camera/color/camera_info      (sensor_msgs/CameraInfo)
  - /detections                           (apriltag_msgs/AprilTagDetectionArray)

Publishes:
  - /apriltag_overlay/image_raw           (sensor_msgs/Image)

The node reprojects the 3D tag corners from each detection's pose back into
pixel space using the camera intrinsics, then draws colored polygons and ID
labels onto a copy of the color image.  The result can be viewed in RViz2
with an Image display subscribed to /apriltag_overlay/image_raw.

Usage in RViz2:
  Add -> By topic -> /apriltag_overlay/image_raw -> Image

Dependencies:
  ros-humble-apriltag-msgs  (or ros-jazzy-apriltag-msgs)
  python3-opencv
  python3-cv-bridge
  python3-numpy
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from message_filters import ApproximateTimeSynchronizer, Subscriber

import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge


# Color cycle for tag outlines (BGR).  Cycles by tag ID so each tag always
# gets the same color within a run.
_COLORS = [
    (0,   220,  80),   # green
    (0,   160, 240),   # blue
    (220,  80,   0),   # orange
    (200,   0, 200),   # magenta
    (0,   200, 200),   # cyan
    (220, 220,   0),   # yellow
]


class AprilTagOverlayNode(Node):

    def __init__(self, save_debug_images=False, exit_after_one=False):
        super().__init__('apriltag_overlay_node')

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter('image_topic',
                               '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic',
                               '/camera/camera/color/camera_info')
        self.declare_parameter('detections_topic',
                               '/detections')
        self.declare_parameter('output_topic',
                               '/apriltag_overlay/image_raw')
        self.declare_parameter('line_thickness', 2)
        self.declare_parameter('font_scale', 0.7)
        self.declare_parameter('corner_radius', 5)
        # Physical half-size of the tag in metres.  Set to the actual printed
        # tag size so the polygon fits the tag precisely.
        self.declare_parameter('tag_half_size', 0.08)

        image_topic        = self.get_parameter('image_topic').value
        camera_info_topic  = self.get_parameter('camera_info_topic').value
        detections_topic   = self.get_parameter('detections_topic').value
        output_topic       = self.get_parameter('output_topic').value
        self._thickness    = self.get_parameter('line_thickness').value
        self._font_scale   = self.get_parameter('font_scale').value
        self._corner_r     = self.get_parameter('corner_radius').value
        self._half_size    = self.get_parameter('tag_half_size').value

        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None

        self.save_debug_images = save_debug_images
        self.exit_after_one = exit_after_one

        # ── QoS ───────────────────────────────────────────────────────────────
        # apriltag_ros publishes RELIABLE; the RealSense driver also publishes
        # RELIABLE for camera_info but BEST_EFFORT for images on some setups.
        # Using BEST_EFFORT here accepts both.
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── synchronised image + detections ───────────────────────────────────
        self._img_sub = Subscriber(self, Image, image_topic,
                                   qos_profile=best_effort_qos)
        self._det_sub = Subscriber(self, AprilTagDetectionArray,
                                   detections_topic,
                                   qos_profile=reliable_qos)

        self._sync = ApproximateTimeSynchronizer(
            [self._img_sub, self._det_sub],
            queue_size=10,
            slop=0.05,          # 50 ms tolerance
        )
        self._sync.registerCallback(self._on_synced)

        # ── publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(Image, output_topic, reliable_qos)

        self.get_logger().info(
            f'apriltag_overlay_node ready\n'
            f'  image:      {image_topic}\n'
            f'  detections: {detections_topic}\n'
            f'  output:     {output_topic}'
        )

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _on_camera_info(self, msg: CameraInfo):
        if self._camera_info is None:
            self.get_logger().info('Camera intrinsics received.')
        self._camera_info = msg

    def _on_synced(self, img_msg: Image,
                   det_msg: AprilTagDetectionArray):
        try:
            frame = self._bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        overlay = frame.copy()

        for det in det_msg.detections:
            tag_id = det.id
            color = _COLORS[tag_id % len(_COLORS)]

            # Use the 2D corners directly from the detection message
            pts_2d = np.array([[corner.x, corner.y] for corner in det.corners], dtype=np.int32)

            # ── filled semi-transparent polygon ──────────────────────────────
            poly_mask = np.zeros_like(frame)
            cv2.fillPoly(poly_mask, [pts_2d], color)
            cv2.addWeighted(overlay, 1.0, poly_mask, 0.25, 0, overlay)

            # ── border ───────────────────────────────────────────────────────
            cv2.polylines(overlay, [pts_2d], isClosed=True,
                          color=color, thickness=self._thickness,
                          lineType=cv2.LINE_AA)

            # ── corner dots ──────────────────────────────────────────────────
            for pt in pts_2d:
                cv2.circle(overlay, tuple(pt), self._corner_r,
                           color, -1, lineType=cv2.LINE_AA)

            # ── ID label ─────────────────────────────────────────────────────
            centroid = pts_2d.mean(axis=0).astype(int)
            label = f'id:{tag_id}'
            (tw, th), bl = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, self._font_scale, 2)
            lx = centroid[0] - tw // 2
            ly = centroid[1] + th // 2

            # Dark background pill for legibility
            pad = 4
            cv2.rectangle(overlay,
                          (lx - pad, ly - th - pad),
                          (lx + tw + pad, ly + pad),
                          (20, 20, 20), -1)
            cv2.putText(overlay, label,
                        (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        self._font_scale, color, 2,
                        lineType=cv2.LINE_AA)

            if self.save_debug_images:
                cv2.imwrite(f'./tag_{tag_id}_overlay.jpg', overlay)
                cv2.imwrite(f'./tag_{tag_id}_original.jpg', frame)

        # ── publish ───────────────────────────────────────────────────────────
        out_msg = self._bridge.cv2_to_imgmsg(overlay, 'bgr8')
        out_msg.header = img_msg.header   # preserve timestamp + frame_id
        self.get_logger().info(f'Publishing overlay with {len(det_msg.detections)} detections.')
        self._pub.publish(out_msg)
        if self.exit_after_one:
            self.get_logger().info('Exiting after processing one frame (exit_after_one=True).')
            exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    # # Init anonymous node, run node to take one image from the stream and annotate it, save both original and annotated images to disk, then exit.  Useful for testing and debugging without RViz2.
    # rclpy.init(args=None)
    # node = AprilTagOverlayNode(save_debug_images=True, exit_after_one=True)
    # try:
    #     # Wait for one camera_info message to get intrinsics, then one image + detections pair.
    #     while node._camera_info is None:
    #         rclpy.spin_once(node, timeout_sec=0.1)
    #     while True:
    #         rclpy.spin_once(node, timeout_sec=0.1)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()