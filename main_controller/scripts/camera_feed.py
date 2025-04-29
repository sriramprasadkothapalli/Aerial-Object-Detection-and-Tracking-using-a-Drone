#!/usr/bin/env python3
"""
front_cam_viewer.py  –  Display /demo/front/image_raw in a small OpenCV window
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class FrontCamViewer(Node):
    def __init__(self):
        super().__init__("front_cam_viewer")

        # Use the sensor-data QoS profile (images arrive fast, best-effort is fine)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.subscription = self.create_subscription(
            Image,
            "/demo/front/image_raw",
            self._img_cb,
            qos,
        )

        self._bridge = CvBridge()

        # Create a named window once; make it small & resizable
        cv2.namedWindow("Front Camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Front Camera", 480, 360)

        self.get_logger().info("Front-camera viewer started ✔")

    # --------------------------------------------------------------------- #
    def _img_cb(self, msg: Image) -> None:
        """Convert ROS Image → OpenCV image and display."""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow("Front Camera", frame)
            cv2.waitKey(1)          # refresh the window
        except Exception as e:
            self.get_logger().error(f"CV conversion failed: {e}")

    # --------------------------------------------------------------------- #
    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()


# ------------------------------------------------------------------------- #
def main() -> None:
    rclpy.init()
    node = FrontCamViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
