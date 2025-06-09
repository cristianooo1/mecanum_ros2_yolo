#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
import math

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        
        # Initialize class variables
        self.odom = None
        self.path = None
        self.count = 0
        
        # Define waypoint threshold (distance to consider waypoint as reached)
        self.waypoint_threshold = 0.05  # meters
        
        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ddd/odom',
            self.odom_callback,
            10)
            
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10)
            
        # Create publisher
        self.vel_pub = self.create_publisher(
            Twist,
            '/ddd/cmd_vel',
            10)
            
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
    def odom_callback(self, msg):
        self.odom = msg
        
    def path_callback(self, msg):
        self.path = msg
        self.count = 0  # Reset waypoint counter when new path is received
        
    def control_loop(self):
        if self.odom is None or self.path is None:
            return
            
        # Check if we've reached the end of the path
        if self.count >= len(self.path.poses):
            # Path completed, stop the robot
            vel_msg = Twist()
            self.vel_pub.publish(vel_msg)
            return
            
        # Get current pose
        pose_x = self.odom.pose.pose.position.x
        pose_y = self.odom.pose.pose.position.y
        
        # Get target waypoint
        goal = self.path.poses[self.count].pose
        target_x = goal.position.x
        target_y = goal.position.y
        
        # Calculate distance to target
        delta_x = target_x - pose_x
        delta_y = target_y - pose_y
        distance = math.sqrt(delta_x**2 + delta_y**2)
        
        # Check if we've reached the current waypoint
        if distance < self.waypoint_threshold:
            self.count += 1
            print(f"Reached waypoint {self.count-1}")
            
            # If we've reached the end of the path
            if self.count >= len(self.path.poses):
                print("Path completed!")
                vel_msg = Twist()
                self.vel_pub.publish(vel_msg)
                return
                
            # Update target to the next waypoint
            goal = self.path.poses[self.count].pose
            target_x = goal.position.x
            target_y = goal.position.y
            delta_x = target_x - pose_x
            delta_y = target_y - pose_y
            
        # Compute control commands (without angular velocity)
        Kx = 1.5
        Ky = 1.5
        
        # Create and send velocity command
        vel_msg = Twist()
        vel_msg.linear.x = Kx * delta_x
        vel_msg.linear.y = Ky * delta_y
        vel_msg.angular.z = 0.0  # No rotation as requested
        
        self.vel_pub.publish(vel_msg)
        print(f"Current pos: ({pose_x:.2f}, {pose_y:.2f}), Target: ({target_x:.2f}, {target_y:.2f}), Distance: {distance:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()