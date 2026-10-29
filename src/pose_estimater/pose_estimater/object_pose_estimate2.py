import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion
from tf2_ros import TransformListener, Buffer, TransformException
from tf2_geometry_msgs import do_transform_point
import numpy as np
from threading import Lock
import math

class PointEstimatorNode(Node):
    def __init__(self):
        super().__init__('point_estimator_node')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_depth_optical_frame')
        self.declare_parameter('transform_timeout', 1.0)
        self.declare_parameter('search_radius', 10)  
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.transform_timeout = self.get_parameter('transform_timeout').get_parameter_value().double_value
        self.search_radius = self.get_parameter('search_radius').get_parameter_value().integer_value

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.cloud_callback,
            10
        )
        self.pixel_sub = self.create_subscription(
            Point,
            '/pixel_coordinates',
            self.pixel_callback,
            10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/object_pose',
            10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_cloud = None
        self.cloud_lock = Lock()
        self.get_logger().info('PointEstimatorNode initialized.')

    def cloud_callback(self, msg):
        # self.get_logger().info('Received new point cloud data.')
        with self.cloud_lock:
            if msg.header.frame_id != self.camera_frame:
                return
            self.latest_cloud = msg

    def get_average_point(self, points, u, v, width, height, radius):
        self.get_logger().info(f"Searching for valid points around pixel ({u}, {v}) with radius {radius}")
        valid_points = []
        for dv in range(-radius, radius + 1):
            for du in range(-radius, radius + 1):
                if du == 0 and dv == 0:
                    continue
                u_neighbor = u + du
                v_neighbor = v + dv
                if u_neighbor < 0 or u_neighbor >= width or v_neighbor < 0 or v_neighbor >= height:
                    continue
                index = v_neighbor * width + u_neighbor
                if index >= len(points):
                    continue
                point = points[index]
                if not any(np.isnan(coord) for coord in point):
                    valid_points.append(point)
        
        if not valid_points:
            return None
        valid_points = np.array(valid_points)
        avg_point = np.mean(valid_points, axis=0)
        return avg_point

    def direction_to_rpy_quaternion(self, direction):
        d = np.array(direction, dtype=float)
        norm = np.linalg.norm(d)
        if norm < 1e-6:
            return Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        d = d / norm
        
        dx, dy, dz = d
        
        if dy> 0 and dx > 0 and dz > 0:
            yaw = np.arctan2(dy, dx)
            pitch = -np.arcsin(dz)
        elif dy< 0 and dx > 0  and dz > 0:
            yaw = -np.arctan2(-dy, dx)
            pitch = np.arcsin(-dz)
        elif dy< 0 and dx < 0  and dz > 0:
            yaw = np.arctan2(-dy, -dx) + math.pi
            pitch = np.arcsin(-dz)
        elif dy> 0 and dx < 0  and dz > 0:
            yaw = -np.arctan2(dy, -dx) - math.pi
            pitch = -np.arcsin(dz)
        elif dy> 0 and dx > 0  and dz < 0:
            yaw = -np.arctan2(dy, -dx) + math.pi
            pitch = np.arcsin(-dz)
        elif dy< 0 and dx > 0  and dz < 0:
            yaw = np.arctan2(-dy, -dx) - math.pi
            pitch = -np.arcsin(dz)
        elif dy< 0 and dx < 0  and dz < 0:
            yaw = -np.arctan2(dx, dy)+ math.pi/2
            pitch = -np.arcsin(dz)
        elif dy> 0 and dx < 0  and dz < 0: 
            yaw = np.arctan2(dx, -dy)- math.pi/2
            pitch = np.arcsin(-dz)
        else:
            yaw = 0.0
            pitch = 0.0
        
        
            
        roll = 0.0
        
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def pixel_callback(self, pixel_msg: Point):
        self.get_logger().info(f'Received pixel coordinates: ({pixel_msg.x}, {pixel_msg.y})')
        with self.cloud_lock:
            if self.latest_cloud is None:
                return
            
            if not (pixel_msg.x >= 0 and pixel_msg.y >= 0):
                return
            u = int(round(pixel_msg.x))
            v = int(round(pixel_msg.y*0.95))
            width = self.latest_cloud.width
            height = self.latest_cloud.height

            if u < 0 or u >= width or v < 0 or v >= height:
                return
            index = v * width + u
            try:
                points = list(pc2.read_points(self.latest_cloud, field_names=["x", "y", "z"], skip_nans=True))
                if index >= len(points):
                    return
                point = points[index]
                if any(np.isnan(coord) for coord in point):
                    point = self.get_average_point(points, u, v, width, height, self.search_radius)
                    if point is None:
                        return 
                point_stamped = PointStamped()
                point_stamped.header = self.latest_cloud.header
                point_stamped.point.x = float(point[0]*0.5)
                point_stamped.point.y = float(point[1]*0.5)
                point_stamped.point.z = float(point[2]*0.5)
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.base_frame,
                        self.camera_frame,
                        rclpy.time.Time(),
                        rclpy.duration.Duration(seconds=self.transform_timeout)
                    )
                    transformed_point = do_transform_point(point_stamped, transform)
                    
                    camera_pos = np.array([
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z
                    ])
                    object_pos = np.array([
                        transformed_point.point.x,
                        transformed_point.point.y,
                        transformed_point.point.z
                    ])
                    direction = object_pos - camera_pos
                    orientation = self.direction_to_rpy_quaternion(direction)
                    
                    pose = PoseStamped()
                    pose.header = transformed_point.header
                    pose.pose.position = transformed_point.point
                    pose.pose.orientation = orientation
                    #input("Press Enter to publish the pose...")
                    self.pose_pub.publish(pose)
                except TransformException:
                    pass
            except StopIteration:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = PointEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()