#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class SimFleetCommander(Node):
    def __init__(self):
        super().__init__('sim_fleet_commander')
        
        # 1. Match the names in your launch file
        self.robot_list = ['burger1', 'burger2', 'burger3']
        
        # 2. Define goals (Make sure they are within the empty world bounds)
        self.goals = {
            'burger1': (0.0, 0.0),
            'burger2': (0.0, 1.0),   # Moving from 10,10 to 8,8
            'burger3': (0.0, -1.0), # Moving from -10,-10 to -8,-8
        }
        
        self.publishers_ = {}

        # 3. Create publishers
        for robot_name in self.robot_list:
            topic_name = f'/{robot_name}/robot_goal'
            self.publishers_[robot_name] = self.create_publisher(Point, topic_name, 10)
            self.get_logger().info(f'Commander ready for: {topic_name}')

        # 4. Wait 2 seconds to ensure connections, then send
        self.timer = self.create_timer(2.0, self.send_goals)

    def send_goals(self):
        self.get_logger().info("--- SENDING GOALS ---")
        for robot_name, (gx, gy) in self.goals.items():
            if robot_name in self.publishers_:
                msg = Point()
                msg.x = float(gx)
                msg.y = float(gy)
                msg.z = 0.0
                
                self.publishers_[robot_name].publish(msg)
                self.get_logger().info(f'>> Sent {robot_name} to ({gx}, {gy})')
        
        self.get_logger().info("---------------------")
        # Exit after sending
        self.timer.cancel()
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = SimFleetCommander()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()