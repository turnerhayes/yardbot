#!/usr/bin/env python3
"""
yardbot_bringup/yardbot_bringup/qos_relay_node.py

Bridges a RELIABLE camera publisher to BEST_EFFORT consumers.

The RealSense driver publishes image_raw as RELIABLE + KEEP_LAST(1).
FastDDS silently drops messages to BEST_EFFORT subscribers from such
publishers (a known DDS quirk).  This node subscribes RELIABLY so it
receives every frame, then republishes as BEST_EFFORT so that nodes
like apriltag_ros (which subscribe BEST_EFFORT via image_transport)
receive a continuous stream.

Subscribed topics  (RELIABLE):
  ~/in/image       sensor_msgs/Image
  ~/in/camera_info sensor_msgs/CameraInfo

Published topics   (BEST_EFFORT):
  ~/out/image       sensor_msgs/Image
  ~/out/camera_info sensor_msgs/CameraInfo
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
)
from sensor_msgs.msg import Image, CameraInfo


RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class QosRelayNode(Node):
    def __init__(self):
        super().__init__("qos_relay")

        self.pub_image = self.create_publisher(
            Image, "~/out/image", BEST_EFFORT_QOS)
        self.pub_info = self.create_publisher(
            CameraInfo, "~/out/camera_info", BEST_EFFORT_QOS)

        self.sub_image = self.create_subscription(
            Image, "~/in/image", self._image_cb, RELIABLE_QOS)
        self.sub_info = self.create_subscription(
            CameraInfo, "~/in/camera_info", self._info_cb, RELIABLE_QOS)

        self.get_logger().info("QoS relay started (RELIABLE -> BEST_EFFORT)")

    def _image_cb(self, msg: Image):
        self.pub_image.publish(msg)

    def _info_cb(self, msg: CameraInfo):
        self.pub_info.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = QosRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
