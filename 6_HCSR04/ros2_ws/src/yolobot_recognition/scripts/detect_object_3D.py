#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from yolov8_msgs.msg import Yolov8Inference
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker
import numpy as np
import cv2

class DetectObject3d(Node):
    def __init__(self):
        super().__init__('detect_object_3d')
        self.get_logger().info('Estimating 3D positions from YOLO detections')
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.detection_callback,
            10)
            
        # Publishers
        self.object3d_pub = self.create_publisher(PointStamped, '/detected_object_3d', 1)
        self.marker_pub = self.create_publisher(Marker, '/object_marker', 1)
        
        # Parameters
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('object_size', 0.2)  # Estimated real size of object (meters)
        self.declare_parameter('camera_frame', 'camera_link_optical')
        
        # Camera calibration parameters
        self.declare_parameter('camera_matrix', [
            575.67874, 0.0, 302.39727,
            0.0, 574.01341, 180.06231,
            0.0, 0.0, 1.0
        ])
        self.declare_parameter('distortion_coeffs', [-0.134926, 0.153370, -0.023055, -0.012495, 0.0])
        
        # Get parameters
        self.img_width = self.get_parameter('image_width').value
        self.img_height = self.get_parameter('image_height').value
        self.object_size = self.get_parameter('object_size').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        # Camera calibration matrix
        cam_matrix = self.get_parameter('camera_matrix').value
        self.camera_matrix = np.array(cam_matrix).reshape(3, 3)
        self.distortion_coeffs = np.array(self.get_parameter('distortion_coeffs').value)
        
        # Compute focal length from camera matrix
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]
        
        self.get_logger().info(f'Camera matrix: {self.camera_matrix}')
        self.get_logger().info(f'Distortion coefficients: {self.distortion_coeffs}')
        
    def detection_callback(self, msg: Yolov8Inference):
        detections = msg.yolov8_inference
        if not detections:
            return
            
        # Process the first detection (or you could filter by class name)
        detection = detections[0]
        
        # Calculate center and size from bounding box
        center_x = (detection.left + detection.right) / 2
        center_y = (detection.top + detection.bottom) / 2
        obj_width = detection.right - detection.left
        obj_height = detection.bottom - detection.top
        
        # Log detection details
        self.get_logger().debug(f'Detection: center=({center_x}, {center_y}), size=({obj_width}, {obj_height})')
        
        # Undistort the point if needed
        if np.any(self.distortion_coeffs != 0):
            points = np.array([[center_x, center_y]], dtype=np.float32)
            undistorted = cv2.undistortPoints(points, self.camera_matrix, self.distortion_coeffs, P=self.camera_matrix)
            center_x, center_y = undistorted[0, 0]
            self.get_logger().debug(f'Undistorted center: ({center_x}, {center_y})')
        
        # Use the pinhole camera model for depth estimation
        # Z = (f * real_size) / pixel_size
        # Using the average of width and height measurements for robustness
        pixel_size = max(obj_width, obj_height)  # Use the larger dimension
        z = (self.fx * self.object_size) / pixel_size
        
        # Calculate X and Y using the pinhole model
        # X = (x - cx) * Z / fx
        # Y = (y - cy) * Z / fy
        x = (center_x - self.cx) * z / self.fx
        y = (center_y - self.cy) * z / self.fy
        
        # In ROS optical frame, Z is forward, X is right, Y is down
        # But we might want standard frame where X is forward, Y is left, Z is up
        # So we'll swap the coordinates
        # Note: Comment/uncomment this section depending on your desired frame convention
        # temp_z = z
        # z = x
        # x = temp_z
        # y = -y
        x_new = z
        z_new = -y
        y_new = -x

        # Publish 3D point
        point_stamped = PointStamped()
        point_stamped.header.frame_id = self.camera_frame
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.point.x = x_new
        point_stamped.point.y = y_new
        point_stamped.point.z = z_new
        self.object3d_pub.publish(point_stamped)
        
        # Create and publish marker
        m = Marker()
        m.header.frame_id = self.camera_frame
        m.header.stamp = self.get_clock().now().to_msg()
        
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.scale.x = self.object_size
        m.scale.y = self.object_size
        m.scale.z = self.object_size
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0
        
        self.marker_pub.publish(m)
        self.get_logger().info(f'Object 3D position: (x:{x_new:.2f}, y:{y_new:.2f}, z:{z_new:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = DetectObject3d()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()