
"""
import rclpy
from rclpy.node import Node
import re
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import SetPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class GlobalInitializer(Node):
    def __init__(self):
        super().__init__('global_initializer')
        print(">>> [SYSTEM] Starting Dynamic Service-Based Initializer...", flush=True)
        
        self.handled_robots = set()
        
        # QoS for OptiTrack (Best Effort)
        self.mocap_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.timer = self.create_timer(1.0, self.discover_robots)

    def discover_robots(self):
        topic_list = self.get_topic_names_and_types()
        pattern = r'^/tb(\d+)/pose$'
        
        for name, _ in topic_list:
            match = re.match(pattern, name)
            if match:
                robot_id = match.group(1)
                if robot_id not in self.handled_robots:
                    print(f">>> [FOUND] Robot {robot_id} on {name}", flush=True)
                    self.call_set_pose_service(robot_id, name)
                    self.handled_robots.add(robot_id)

    def call_set_pose_service(self, robot_id, source_topic):
        # Create a client for the SetPose service
        srv_name = f'/tb{robot_id}/set_pose'
        client = self.create_client(SetPose, srv_name)
        
        while not client.wait_for_service(timeout_sec=1.0):
            print(f"Waiting for service {srv_name}...", flush=True)

        def mocap_callback(msg):
            print(f">>> [DATA] Received OptiTrack pose for Robot {robot_id}. Calling Service...", flush=True)
            
            request = SetPose.Request()
            request.pose.header = msg.header
            request.pose.header.frame_id = 'map' # Ensure this matches EKF world_frame
            request.pose.pose.pose = msg.pose
            
            # Add basic covariance
            cov = [0.0] * 36
            for i in [0, 7, 14, 21, 28, 35]: cov[i] = 0.01
            request.pose.pose.covariance = cov
            
            # Call service asynchronously
            client.call_async(request)
            print(f">>> [SUCCESS] Service call sent for Robot {robot_id}!", flush=True)
            
            # Clean up: stop listening to OptiTrack for this robot
            self.destroy_subscription(sub)

        sub = self.create_subscription(PoseStamped, source_topic, mocap_callback, self.mocap_qos)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalInitializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
import rclpy
from rclpy.node import Node
import re
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import SetPose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class GlobalInitializer(Node):
    def __init__(self):
        super().__init__('global_initializer')
        # Use a Reentrant group so the timer and service calls can run in parallel
        self.callback_group = ReentrantCallbackGroup()
        
        print(">>> [SYSTEM] Starting Multithreaded Initializer...", flush=True)
        
        self.handled_robots = set()
        
        self.mocap_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Attach timer to the callback group
        self.timer = self.create_timer(1.0, self.discover_robots, callback_group=self.callback_group)

    def discover_robots(self):
        topic_list = self.get_topic_names_and_types()
        pattern = r'^/tb(\d+)/pose$'
        
        for name, _ in topic_list:
            match = re.match(pattern, name)
            if match:
                robot_id = match.group(1)
                if robot_id not in self.handled_robots:
                    print(f">>> [FOUND] Robot {robot_id} on {name}", flush=True)
                    self.handled_robots.add(robot_id)
                    self.call_set_pose_service(robot_id, name)

    def call_set_pose_service(self, robot_id, source_topic):
        srv_name = f'/tb{robot_id}/set_pose'
        # Attach client to the callback group
        client = self.create_client(SetPose, srv_name, callback_group=self.callback_group)
        
        # Wait for service (Non-blocking for other threads)
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"[WARN] Service {srv_name} not responding. Will retry...", flush=True)
            self.handled_robots.remove(robot_id)
            return

        def mocap_callback(msg):
            print(f">>> [DATA] Received OptiTrack for TB{robot_id}. Sending service request...", flush=True)
            
            request = SetPose.Request()
            request.pose.header = msg.header
            request.pose.header.frame_id = 'map'
            
            # Position and Orientation
            request.pose.pose.pose.position.x = msg.pose.position.x
            request.pose.pose.pose.position.y = msg.pose.position.y
            request.pose.pose.pose.position.z = 0.0 
            request.pose.pose.pose.orientation = msg.pose.orientation
            
            # Covariance
            cov = [0.0] * 36
            for i in [0, 7, 14, 21, 28, 35]: cov[i] = 0.01
            request.pose.pose.covariance = cov
            
            future = client.call_async(request)
            future.add_done_callback(lambda f: print(f">>> [SUCCESS] TB{robot_id} initialized!", flush=True))
            
            # Clean up subscription after successful capture
            self.destroy_subscription(sub)

        # Attach subscription to the callback group
        sub = self.create_subscription(
            PoseStamped, 
            source_topic, 
            mocap_callback, 
            self.mocap_qos,
            callback_group=self.callback_group
        )

def main(args=None):
    rclpy.init(args=args)
    node = GlobalInitializer()
    
    # Use MultiThreadedExecutor to handle the callbacks in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()