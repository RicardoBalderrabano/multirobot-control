#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class FleetCommander(Node):
    def __init__(self):
        super().__init__('fleet_commander')
        
        # 1. Define your fleet (Must match the names in your Launch File)
        self.robot_list = ['tb1', 'tb3', 'tb6']
        
        # 2. Define goals for each robot (x, y)
        self.goals = {
            'tb1': (-1.0, -1.0),
            'tb3': (1.0, -1.0),
            'tb6': (1.0, 1.0),
        
        }
        
        self.publishers_ = {}

        # 3. Create a publisher for each robot
        for robot_name in self.robot_list:
            topic_name = f'/{robot_name}/robot_goal'
            self.publishers_[robot_name] = self.create_publisher(Point, topic_name, 10)
            self.get_logger().info(f'Registered commander for: {topic_name}')

        # 4. Wait briefly for connections, then send
        self.timer = self.create_timer(1.0, self.send_goals)

    def send_goals(self):
        for robot_name, (gx, gy) in self.goals.items():
            if robot_name in self.publishers_:
                msg = Point()
                msg.x = float(gx)
                msg.y = float(gy)
                msg.z = 0.0
                
                self.publishers_[robot_name].publish(msg)
                self.get_logger().info(f'Sent goal {gx, gy} to {robot_name}')
        
        # Shut down after sending once
        self.timer.cancel()
        self.get_logger().info("All goals sent. Shutting down.")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = FleetCommander()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()