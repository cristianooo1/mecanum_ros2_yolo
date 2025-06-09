#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
import math

class ObjectFollower(Node):
    def __init__(self):
        super().__init__('object_follower')
        
        # Subscriber for 3D position of detected object
        self.position_sub = self.create_subscription(
            PointStamped,
            '/detected_object_3d',
            self.position_callback,
            10)
            
        # Publisher for robot velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/ddd/cmd_vel',
            10)
            
        # Parameters for rotation control
        self.declare_parameter('max_angular_speed', 0.2)
        self.declare_parameter('min_angular_speed', 0.07)
        self.declare_parameter('alignment_threshold', 0.2)
        self.declare_parameter('deceleration_factor', 0.9)
        self.declare_parameter('no_detection_timeout', 0.5)
        self.declare_parameter('velocity_decay_rate', 0.9)
        
        # Parameters for forward movement
        self.declare_parameter('max_linear_speed', 0.2)
        self.declare_parameter('min_linear_speed', 0.07)
        self.declare_parameter('target_distance', 0.15)  # Target distance to maintain from object (meters)
        self.declare_parameter('distance_threshold', 0.15)  # How close to get to target distance
        
        # Get rotation parameters
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_angular_speed = self.get_parameter('min_angular_speed').value
        self.alignment_threshold = self.get_parameter('alignment_threshold').value
        self.deceleration_factor = self.get_parameter('deceleration_factor').value
        self.no_detection_timeout = self.get_parameter('no_detection_timeout').value
        self.velocity_decay_rate = self.get_parameter('velocity_decay_rate').value
        
        # Get forward movement parameters
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.target_distance = self.get_parameter('target_distance').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        
        # State variables
        self.current_angular_velocity = 0.0
        self.current_linear_velocity = 0.0
        self.last_detection_time = None
        self.last_angle_to_object = 0.0
        self.is_aligned = False
        
        # Create a timer for velocity decay when no detections are received
        self.decay_timer = self.create_timer(0.1, self.decay_velocity)
        
        self.get_logger().info('Complete object follower initialized')
        
    def position_callback(self, msg: PointStamped):
        # Update last detection time
        self.last_detection_time = self.get_clock().now()
        
        # Extract position from the message
        x = msg.point.x  # forward direction
        y = msg.point.y  # left/right direction
        z = msg.point.z  # height
        
        # Calculate distance to object (in the horizontal plane)
        distance_to_object = math.sqrt(x*x + y*y)
        
        # Calculate angle to the object in the horizontal plane
        angle_to_object = math.atan2(y, x)
        
        # Calculate angle change rate for damping
        angle_change = abs(angle_to_object - self.last_angle_to_object)
        self.last_angle_to_object = angle_to_object
        
        self.get_logger().debug(f'Object at: x={x:.2f}, y={y:.2f}, z={z:.2f}, distance={distance_to_object:.2f}m, angle={angle_to_object:.4f}rad')
        
        # Create Twist message
        twist = Twist()
        
        # First priority: Check alignment
        if abs(angle_to_object) > self.alignment_threshold:
            # Object is not aligned, need to rotate
            self.is_aligned = False
            direction = 1.0 if angle_to_object > 0 else -1.0
            
            # Calculate target angular velocity with proportional control and damping
            target_velocity = direction * min(
                self.max_angular_speed, 
                max(self.min_angular_speed, abs(angle_to_object * 1.5))
            )
            
            # Apply damping if angle is decreasing (approaching alignment)
            if angle_change < 0.01:
                target_velocity *= self.deceleration_factor
                
            # Smooth transition to new velocity
            alpha = 0.7
            self.current_angular_velocity = alpha * target_velocity + (1-alpha) * self.current_angular_velocity
            
            # When rotating to align, stop forward movement
            self.current_linear_velocity = 0.0
            
            twist.angular.z = self.current_angular_velocity
            self.get_logger().info(f'Aligning: angle={angle_to_object:.2f}, angular_vel={twist.angular.z:.2f}')
        else:
            # Object is aligned with robot's forward direction
            self.is_aligned = True
            
            # Gradually reduce angular velocity to zero
            self.current_angular_velocity *= 0.5
            if abs(self.current_angular_velocity) < 0.01:
                self.current_angular_velocity = 0.0
                
            twist.angular.z = self.current_angular_velocity
            
            # Object is aligned, now handle distance control (forward/backward movement)
            distance_error = distance_to_object - self.target_distance
            
            if abs(distance_error) > self.distance_threshold:
                # Need to move to achieve target distance
                direction = 1.0 if distance_error > 0 else -1.0  # Forward if too far, backward if too close
                
                # Proportional control for linear velocity
                target_linear_velocity = direction * min(
                    self.max_linear_speed,
                    max(self.min_linear_speed, abs(distance_error * 0.3))  # Proportional to distance error
                )
                
                # Smooth transition for linear velocity
                alpha = 0.7
                self.current_linear_velocity = alpha * target_linear_velocity + (1-alpha) * self.current_linear_velocity
                
                twist.linear.x = self.current_linear_velocity
                self.get_logger().info(f'Moving: distance={distance_to_object:.2f}m, target={self.target_distance:.2f}m, linear_vel={twist.linear.x:.2f}')
            else:
                # At target distance, stop
                self.current_linear_velocity *= 0.5
                if abs(self.current_linear_velocity) < 0.01:
                    self.current_linear_velocity = 0.0
                    
                twist.linear.x = self.current_linear_velocity
                self.get_logger().info('At target position')
            
        # Publish the twist message
        self.cmd_vel_pub.publish(twist)
        
    def decay_velocity(self):
        """Gradually decay velocity if no recent detections"""
        if self.last_detection_time is None:
            return
            
        # Calculate time since last detection
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_detection_time).nanoseconds / 1e9
        
        # If it's been too long since last detection, decay both velocities
        if time_diff > self.no_detection_timeout:
            velocity_changed = False
            
            # Decay angular velocity if not near zero
            if abs(self.current_angular_velocity) > 0.01:
                self.current_angular_velocity *= self.velocity_decay_rate
                velocity_changed = True
                
            # Decay linear velocity if not near zero
            if abs(self.current_linear_velocity) > 0.01:
                self.current_linear_velocity *= self.velocity_decay_rate
                velocity_changed = True
                
            # Only publish if velocity actually changed
            if velocity_changed:
                twist = Twist()
                twist.angular.z = self.current_angular_velocity
                twist.linear.x = self.current_linear_velocity
                self.cmd_vel_pub.publish(twist)
                
                self.get_logger().debug(
                    f'No detection for {time_diff:.2f}s, '
                    f'decaying velocities to linear={self.current_linear_velocity:.2f}, '
                    f'angular={self.current_angular_velocity:.2f}'
                )

def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()