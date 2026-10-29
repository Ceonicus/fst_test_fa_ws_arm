import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from cv_bridge import CvBridge
import struct

class RGBImageExtractorNode(Node):
    def __init__(self):
        super().__init__('rgb_image_extractor_node')
        self.declare_parameter('camera_frame', 'camera_depth_optical_frame')
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.cloud_callback,
            10
        )
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )
        self.bridge = CvBridge()

    def cloud_callback(self, msg: PointCloud2):
        if msg.header.frame_id != self.camera_frame:
            return
        width = msg.width
        height = msg.height
        if width == 0 or height == 0:
            return
        try:
            points = pc2.read_points(msg, field_names=["rgb"], skip_nans=False)
            rgb_data = []
            for point in points:
                rgb_packed = point[0]
                rgb_int = struct.unpack('I', struct.pack('f', rgb_packed))[0] if isinstance(rgb_packed, float) else int(rgb_packed)
                r = (rgb_int >> 16) & 0xFF
                g = (rgb_int >> 8) & 0xFF
                b = rgb_int & 0xFF
                rgb_data.append([r, g, b])
            rgb_array = np.array(rgb_data, dtype=np.uint8).reshape(height, width, 3)
            image_msg = self.bridge.cv2_to_imgmsg(rgb_array, encoding='rgb8')
            image_msg.header = msg.header
            self.image_pub.publish(image_msg)
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RGBImageExtractorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()