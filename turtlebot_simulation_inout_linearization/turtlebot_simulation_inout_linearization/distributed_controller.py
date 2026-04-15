#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point, PoseStamped

class DistributedController(Node):
    def __init__(self):
        super().__init__('distributed_controller')
        
        # --- Parameters from Professor's Instructions ---
        self.alpha = 1.0  # Attraction gain 
        self.beta = 5.0   # Repulsion gain 
        self.robots = ['tb1', 'tb2', 'tb3'] # Robot list [cite: 2]
        
        # Storage for the current positions (Point B) of all robots [cite: 7]
        self.robot_positions = {name: None for name in self.robots}
        
        # Subscriptions: Listen to the pose of every robot in the fleet
        self.subs = []
        for name in self.robots:
            # We use ground_truth_pose as it represents the global x,y 
            topic = f'/{name}/ground_truth_pose'
            sub = self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, n=name: self.pose_callback(msg, n),
                10
            )
            self.subs.append(sub)
            
        # Publishers: Send the calculated velocity vector to each sim_controller
        self.goal_pubs = {
            name: self.create_publisher(Point, f'/{name}/robot_goal', 10)
            for name in self.robots
        }
        
        # Control loop at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Distributed Controller (Attraction-Repulsion) Started")

    def pose_callback(self, msg, name):
        # Store position as a numpy array for vector math [cite: 3, 7]
        self.robot_positions[name] = np.array([
            msg.pose.position.x,
            msg.pose.position.y
        ])

    def control_loop(self):
        # Only proceed if we have data for all robots
        if any(pos is None for pos in self.robot_positions.values()):
            return

        for i_name in self.robots:
            xi = self.robot_positions[i_name]
            
            # Initialize dot_xi (the desired velocity of Point B) 
            dot_xi = np.array([0.0, 0.0])
            
            for j_name in self.robots:
                if i_name == j_name:
                    continue
                
                xj = self.robot_positions[j_name]
                
                # Calculate relative vector (xi - xj) 
                diff = xi - xj
                dist_sq = np.sum(diff**2) # ||xi - xj||^2 
                
                # 1. Attraction Term: -alpha * (xi - xj) 
                attraction = -self.alpha * diff
                
                # 2. Repulsion Term: beta * (diff / dist_sq) 
                # Note: Adding a small epsilon prevents division by zero
                repulsion = self.beta * (diff / (dist_sq + 1e-6))
                
                dot_xi += (attraction + repulsion)
            
            # Convert the velocity into a "Moving Goal" for your existing I/O Controller
            # We project the current position forward by the calculated velocity
            goal_msg = Point()
            goal_msg.x = xi[0] + dot_xi[0] * 0.1 
            goal_msg.y = xi[1] + dot_xi[1] * 0.1
            goal_msg.z = 0.0
            
            self.goal_pubs[i_name].publish(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DistributedController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
