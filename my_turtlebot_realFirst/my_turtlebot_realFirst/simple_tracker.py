import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
import math
import numpy as np

# --- CONFIGURATION ---
SCAN_TOPIC = '/tb1/scan'
DT = 0.1               # Loop rate
GATE_DIST = 0.5        # If a measurement is > 0.5m from a track, it's not that robot
MAX_NO_UPDATE = 5      # Delete track if we haven't seen it for 5 frames
# ---------------------

class Track:
    new_id = 0 

    def __init__(self, x, y):
        self.id = Track.new_id
        Track.new_id += 1
        
        # State
        self.x = x
        self.y = y
        self.vx = 0.0
        self.vy = 0.0
        
        self.skipped_frames = 0 
        
        # --- TUNING FIX 1: Lower Beta to reduce velocity noise ---
        self.alpha = 0.5 
        self.beta  = 0.1  # Was 0.4 (Too high!)

    def predict(self, dt):
        # --- TUNING FIX 2: Decay velocity when lost ---
        # If we haven't seen the robot, assume it's slowing down, not flying away.
        if self.skipped_frames > 0:
            self.vx *= 0.9
            self.vy *= 0.9

        self.x += self.vx * dt
        self.y += self.vy * dt
        self.skipped_frames += 1 

    def update(self, meas_x, meas_y, dt):
        resid_x = meas_x - self.x
        resid_y = meas_y - self.y
        
        # Update Position
        self.x += self.alpha * resid_x
        self.y += self.alpha * resid_y
        
        # Update Velocity
        new_vx = self.vx + (self.beta * resid_x / dt)
        new_vy = self.vy + (self.beta * resid_y / dt)
        
        # --- TUNING FIX 3: Hard Velocity Clamp ---
        # TurtleBots max speed is ~0.22 m/s. We allow 0.5 m/s for safety.
        # If math says 9.0 m/s, we force it down.
        MAX_VEL = 0.5 
        velocity_magnitude = math.hypot(new_vx, new_vy)
        
        if velocity_magnitude > MAX_VEL:
            scale = MAX_VEL / velocity_magnitude
            new_vx *= scale
            new_vy *= scale
            
        self.vx = new_vx
        self.vy = new_vy
        
        self.skipped_frames = 0

class MultiTracker(Node):
    def __init__(self):
        super().__init__('multi_tracker')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self.scan_callback, qos)
        
        self.tracks = [] # List of Track objects
        self.last_time = self.get_clock().now()
        
        self.get_logger().info("Multi-Tracker Started. Waiting for robots...")

    def scan_callback(self, msg):
        # 1. Calc Time Step
        curr_time = self.get_clock().now()
        dt = (curr_time - self.last_time).nanoseconds / 1e9
        if dt == 0: dt = 0.1
        self.last_time = curr_time

        # 2. Get Raw Measurements (Candidates)
        measurements = self.detect_objects(msg)

        # 3. Predict Step (Move all existing tracks forward)
        for t in self.tracks:
            t.predict(dt)

        # 4. Data Association (Match Measurements to Tracks)
        # We use a simple Greedy Nearest Neighbor approach
        
        unassigned_measurements = measurements[:] # Copy list
        
        for track in self.tracks:
            best_dist = GATE_DIST
            best_meas = None
            
            # Find closest measurement to this track
            for m in unassigned_measurements:
                dist = math.hypot(m[0] - track.x, m[1] - track.y)
                if dist < best_dist:
                    best_dist = dist
                    best_meas = m
            
            # If we found a match, update the track and remove measurement
            if best_meas:
                track.update(best_meas[0], best_meas[1], dt)
                unassigned_measurements.remove(best_meas)

        # 5. Management: Create New & Delete Old
        
        # Create new tracks for any measurement that wasn't matched
        for m in unassigned_measurements:
            new_track = Track(m[0], m[1])
            self.tracks.append(new_track)
            self.get_logger().info(f"New Robot Detected! ID: {new_track.id}")

        # Delete dead tracks (ghosts)
        self.tracks = [t for t in self.tracks if t.skipped_frames < MAX_NO_UPDATE]

        # 6. Output (This is what you send to your Control Law)
        print(f"--- FRAME --- Visible: {len(self.tracks)}")
        for t in self.tracks:
            # Only print if we are actively tracking it (skipped_frames=0)
            status = "LOST" if t.skipped_frames > 0 else "TRACKING"
            print(f"ID {t.id} [{status}]: Pos({t.x:.2f}, {t.y:.2f}) Vel({t.vx:.2f}, {t.vy:.2f})")

    def detect_objects(self, msg):
        """ Standard Phase 1 detection code """
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if 0.05 < r < 2.0: # Keep it under 2.0m as requested
                points.append((r * math.cos(angle), r * math.sin(angle)))
            angle += msg.angle_increment
            
        if not points: return []
        
        # Cluster
        clusters = []
        curr = [points[0]]
        for i in range(1, len(points)):
            if math.hypot(points[i][0]-points[i-1][0], points[i][1]-points[i-1][1]) < 0.20:
                curr.append(points[i])
            else:
                clusters.append(curr)
                curr = [points[i]]
        clusters.append(curr)
        
        # Centroids
        centers = []
        for c in clusters:
            if len(c) < 3: continue
            w = math.hypot(c[0][0]-c[-1][0], c[0][1]-c[-1][1])
            if 0.05 < w < 0.40:
                # Filter negative X (behind)
                cx = sum(p[0] for p in c)/len(c)
                cy = sum(p[1] for p in c)/len(c)
                if cx > 0.0:
                    centers.append((cx, cy))
        return centers

def main(args=None):
    rclpy.init(args=args)
    node = MultiTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()