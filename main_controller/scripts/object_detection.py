#!/usr/bin/env python3
"""
front_cam_yolov8.py – run YOLOv8 on /demo/front/image_raw
"""

import time
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np

from ultralytics import YOLO      # pip install ultralytics

CONF_THRES = 0.25                 # detection score threshold
MODEL_PATH = "yolov8n.pt"         # use any YOLOv8 *.pt weights

class YOLOv8Node(Node):

    def __init__(self):
        super().__init__("front_cam_yolov8")
        self.bridge = CvBridge()

        # load model once
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info(f"Loaded {MODEL_PATH} – device: {self.model.device}")

        qos = rclpy.qos.qos_profile_sensor_data
        self.sub = self.create_subscription(
            Image, "/demo/front/image_raw", self.img_cb, qos)

        self.pub_img = self.create_publisher(Image, "/yolo/annotated", 10)
        self.pub_det = self.create_publisher(Detection2DArray, "/yolo/detections", 10)

        cv2.namedWindow("YOLO v8", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("YOLO v8", 640, 480)

        self.last_time = time.time()

    # ----------------------------------------------------------
    def img_cb(self, msg: Image):
        ns = time.time_ns()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Run inference
        results = self.model(frame, verbose=False)[0]      # Ultralytics Result

        annotated = frame.copy()
        det_array = Detection2DArray()
        det_array.header = msg.header

        for box, score, cls in zip(results.boxes.xyxy.cpu().numpy(),
                                   results.boxes.conf.cpu().numpy(),
                                   results.boxes.cls.cpu().numpy()):
            if score < CONF_THRES:
                continue

            x1, y1, x2, y2 = map(int, box)
            label = self.model.names[int(cls)]

            # draw
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, f"{label} {score:.2f}",
                        (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)

            # fill Detection2D
            det = Detection2D()
            det.bbox.center.position.x = float((x1 + x2) / 2.0)
            det.bbox.center.position.y = float((y1 + y2) / 2.0)
            det.bbox.center.theta      = 0.0                # axis-aligned box
            det.bbox.size_x            = float(x2 - x1)
            det.bbox.size_y            = float(y2 - y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(int(cls))   # YOLO class I8D as string
            hyp.hypothesis.score    = float(score)
            det.results.append(hyp)

            det_array.detections.append(det)

        # publish / display
        self.pub_det.publish(det_array)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = msg.header
        self.pub_img.publish(annotated_msg)

        cv2.imshow("YOLO v8", annotated)
        cv2.waitKey(1)

        # perf debug
        fps = 1.0 / (time.time() - self.last_time)
        self.last_time = time.time()
        self.get_logger().debug(f"FPS {fps:.1f}")

    # ----------------------------------------------------------
    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = YOLOv8Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
