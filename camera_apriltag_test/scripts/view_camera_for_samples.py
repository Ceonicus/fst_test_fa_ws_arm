#!/usr/bin/env python3

import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray


class CameraSampleViewer(Node):
    def __init__(self):
        super().__init__("camera_sample_viewer")

        self.bridge = CvBridge()

        self.image_topic = "/camera/camera/color/image_raw"
        self.detection_topic = "/detections"

        self.latest_detections = []
        self.latest_detection_time = 0.0

        self.image_subscriber = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.detection_subscriber = self.create_subscription(
            AprilTagDetectionArray,
            self.detection_topic,
            self.detection_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Camera sample viewer started.")
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"Detection topic: {self.detection_topic}")
        self.get_logger().info("Press q in the image window to quit.")

    def detection_callback(self, message):
        self.latest_detections = list(message.detections)
        self.latest_detection_time = time.time()

    def get_detection_center(self, detection):
        centre = getattr(detection, "centre", None)

        if centre is None:
            centre = getattr(detection, "center", None)

        if centre is None:
            return None

        try:
            return int(centre.x), int(centre.y)
        except Exception:
            return None

    def get_detection_corners(self, detection):
        corners = getattr(detection, "corners", None)

        if corners is None:
            return []

        output = []

        for corner in corners:
            try:
                output.append((int(corner.x), int(corner.y)))
            except Exception:
                pass

        return output

    def get_detection_id(self, detection):
        return getattr(detection, "id", "unknown")

    def get_detection_margin(self, detection):
        try:
            return float(getattr(detection, "decision_margin", 0.0))
        except Exception:
            return 0.0

    def get_detection_hamming(self, detection):
        try:
            return int(getattr(detection, "hamming", -1))
        except Exception:
            return -1

    def draw_text(self, image, text, x, y, scale=0.65, thickness=2):
        cv2.putText(
            image,
            text,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            (255, 255, 255),
            thickness + 2,
            cv2.LINE_AA,
        )

        cv2.putText(
            image,
            text,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            (0, 255, 0),
            thickness,
            cv2.LINE_AA,
        )

    def image_callback(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            self.get_logger().error(f"Failed to convert image: {error}")
            return

        height, width = frame.shape[:2]

        image_center_x = width // 2
        image_center_y = height // 2

        safe_left = int(width * 0.25)
        safe_right = int(width * 0.75)
        safe_top = int(height * 0.20)
        safe_bottom = int(height * 0.80)

        cv2.line(
            frame,
            (image_center_x - 30, image_center_y),
            (image_center_x + 30, image_center_y),
            (255, 255, 255),
            2,
        )

        cv2.line(
            frame,
            (image_center_x, image_center_y - 30),
            (image_center_x, image_center_y + 30),
            (255, 255, 255),
            2,
        )

        cv2.rectangle(
            frame,
            (safe_left, safe_top),
            (safe_right, safe_bottom),
            (255, 255, 0),
            2,
        )

        self.draw_text(
            frame,
            "Safe sample zone",
            safe_left,
            safe_top - 10,
            scale=0.55,
            thickness=1,
        )

        detections_are_fresh = (time.time() - self.latest_detection_time) < 0.5

        status_text = "NO TAG"
        sample_quality_text = "Do not take sample"
        status_color = (0, 0, 255)

        if detections_are_fresh and len(self.latest_detections) > 0:
            detection = self.latest_detections[0]

            tag_center = self.get_detection_center(detection)
            tag_corners = self.get_detection_corners(detection)
            tag_id = self.get_detection_id(detection)
            decision_margin = self.get_detection_margin(detection)
            hamming = self.get_detection_hamming(detection)

            if len(tag_corners) >= 4:
                for index in range(len(tag_corners)):
                    point_1 = tag_corners[index]
                    point_2 = tag_corners[(index + 1) % len(tag_corners)]
                    cv2.line(frame, point_1, point_2, (0, 255, 255), 2)

                for corner in tag_corners:
                    cv2.circle(frame, corner, 4, (0, 255, 255), -1)

            if tag_center is not None:
                tag_x, tag_y = tag_center

                cv2.circle(frame, (tag_x, tag_y), 7, (0, 0, 255), -1)
                cv2.line(
                    frame,
                    (image_center_x, image_center_y),
                    (tag_x, tag_y),
                    (0, 255, 255),
                    2,
                )

                offset_x = tag_x - image_center_x
                offset_y = tag_y - image_center_y

                inside_safe_box = (
                    safe_left <= tag_x <= safe_right
                    and safe_top <= tag_y <= safe_bottom
                )

                good_margin = decision_margin >= 50.0
                good_hamming = hamming == 0

                if inside_safe_box and good_margin and good_hamming:
                    status_text = "TAG OK"
                    sample_quality_text = "GOOD SAMPLE - wait 1-2s, then Take Sample"
                    status_color = (0, 180, 0)
                elif not inside_safe_box:
                    status_text = "TAG NEAR EDGE"
                    sample_quality_text = "Move view so tag is inside the safe box"
                    status_color = (0, 165, 255)
                elif not good_margin:
                    status_text = "LOW DECISION MARGIN"
                    sample_quality_text = "Improve lighting / reduce angle / move closer"
                    status_color = (0, 165, 255)
                elif not good_hamming:
                    status_text = "HAMMING ERROR"
                    sample_quality_text = "Do not sample; tag decode is not clean"
                    status_color = (0, 0, 255)

                self.draw_text(frame, f"Tag ID: {tag_id}", 20, 35)
                self.draw_text(frame, f"margin: {decision_margin:.1f}, hamming: {hamming}", 20, 65)
                self.draw_text(frame, f"offset: x={offset_x:+d}px, y={offset_y:+d}px", 20, 95)

            else:
                status_text = "TAG DETECTED, NO CENTER FIELD"
                sample_quality_text = "Check AprilTag message format"
                status_color = (0, 165, 255)

        cv2.rectangle(frame, (0, height - 70), (width, height), status_color, -1)

        cv2.putText(
            frame,
            status_text,
            (20, height - 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            sample_quality_text,
            (20, height - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        cv2.imshow("RealSense AprilTag Sample Viewer", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            rclpy.shutdown()


def main():
    rclpy.init()

    node = CameraSampleViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
