#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import cv2
from ultralytics import YOLO
import numpy as np

class YoloObjectDetector(Node):
    def __init__(self):
        super().__init__('yolo_object_detector')
        self.declare_parameter('model_path', '/home/fst/test_fa_ws/yolo_custom.pt')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('output_topic', '/yolo/detections')
        self.declare_parameter('confidence_threshold', 0.5)
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, output_topic, 10)
        self.coord_publisher = self.create_publisher(Point, '/pixel_coordinates', 10)

    def image_callback(self, msg):
        try:
            self.get_logger().info('Received image for processing.')
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info(f'Image dimensions: {cv_image.shape[1]}x{cv_image.shape[0]}')  
            results = self.model(cv_image, conf=self.conf_threshold)
            class_names = results[0].names
            target_label = "Interlock w/ Cover" 
            for box, cls in zip(results[0].boxes.xyxy, results[0].boxes.cls):
                class_name = class_names[int(cls)] 
                if class_name == target_label:
                    x_min, y_min, x_max, y_max = box.tolist()
                    center_x = (x_min + x_max) / 2.0
                    center_y = (y_min + y_max) / 2.0
                    point_msg = Point()
                    point_msg.x = center_x
                    point_msg.y = center_y
                    point_msg.z = 0.0
                    self.coord_publisher.publish(point_msg)

                    self.get_logger().info(f'Published pixel coordinates for {target_label}: ({center_x}, {center_y})')

            annotated_image = results[0].plot()
            output_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            output_msg.header = msg.header
            self.publisher.publish(output_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()