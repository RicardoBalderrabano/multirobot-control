#!/usr/bin/env python3
"""
control_strategies.py
Library of Distributed Control Laws for Swarm Robotics.
"""

import numpy as np

class BaseControlLaw:
    """
    Interface that all control strategies must follow.
    Ensures plug-and-play capability in the main ROS 2 node.
    """
    def compute(self, robot_state, neighbors, traj_ref, params):
        raise NotImplementedError("Subclasses must implement compute()")


class ConsensusCollision(BaseControlLaw):
    """
    Mode 1: Basic Consensus and Collision Avoidance.
    No absolute goal; robots just form a geometry and hold it.
    """
    def compute(self, robot_state, neighbors, traj_ref, params):
        u_x, u_y = 0.0, 0.0
        
        for nid, n_pos in neighbors.items():
            dx = robot_state['xB'] - n_pos['x']
            dy = robot_state['yB'] - n_pos['y']
            dist_sq = max(dx**2 + dy**2, 0.0001)
            
            # Isotropic Cohesion and Dispersion
            u_x += -params['alpha'] * dx + params['beta'] * (dx / dist_sq)
            u_y += -params['alpha'] * dy + params['beta'] * (dy / dist_sq)
            
        return u_x, u_y


class AbsoluteGoalTracking(BaseControlLaw):
    """
    Mode 2: Consensus, Collision Avoidance, and Static Goal Tracking.
    Robots form a geometry and travel to a fixed (x, y) coordinate.
    """
    def compute(self, robot_state, neighbors, traj_ref, params):
        u_x, u_y = 0.0, 0.0
        
        # 1. Local APF
        for nid, n_pos in neighbors.items():
            dx = robot_state['xB'] - n_pos['x']
            dy = robot_state['yB'] - n_pos['y']
            dist_sq = max(dx**2 + dy**2, 0.0001)
            
            u_x += -params['alpha'] * dx + params['beta'] * (dx / dist_sq)
            u_y += -params['alpha'] * dy + params['beta'] * (dy / dist_sq)
            
        # 2. Absolute Goal Tracking
        u_x += -params['rho'] * (robot_state['xB'] - params['goal_x'])
        u_y += -params['rho'] * (robot_state['yB'] - params['goal_y'])
        
        return u_x, u_y


class DynamicTrajectory(BaseControlLaw):
    """
    Mode 3: Circular/Figure-8 Tracking with Isotropic Potential Fields.
    Robots track a moving reference point with velocity feedforward.
    """
    def compute(self, robot_state, neighbors, traj_ref, params):
        u_x, u_y = 0.0, 0.0
        
        # 1. Local APF
        for nid, n_pos in neighbors.items():
            dx = robot_state['xB'] - n_pos['x']
            dy = robot_state['yB'] - n_pos['y']
            dist_sq = max(dx**2 + dy**2, 0.0001)
            
            u_x += -params['alpha'] * dx + params['beta'] * (dx / dist_sq)
            u_y += -params['alpha'] * dy + params['beta'] * (dy / dist_sq)
            
        # 2. Dynamic Trajectory Tracking & Feedforward
        if traj_ref is not None:
            r_x, r_y, r_dot_x, r_dot_y = traj_ref
            u_x += -params['rho'] * (robot_state['xB'] - r_x) + r_dot_x
            u_y += -params['rho'] * (robot_state['yB'] - r_y) + r_dot_y
            
        return u_x, u_y


class AnisotropicEllipse(BaseControlLaw):
    def compute(self, robot_state, neighbors, traj_ref, params):
        if traj_ref is None: return 0.0, 0.0
            
        r_x, r_y, r_dot_x, r_dot_y = traj_ref
        
        if params.get('is_rotating', False):
            ellipse_alpha = np.arctan2(r_dot_y, r_dot_x)
        else:
            ellipse_alpha = params.get('ellipse_alpha', 0.0)

        ca, sa = np.cos(ellipse_alpha), np.sin(ellipse_alpha)
        R_mat = np.array([[ca, -sa], [sa, ca]])
        
        # The professor's M matrix handles BOTH shape and repulsion strength!
        M_diag = np.array([[params.get('m11', 1.0), 1.0], 
                           [-1.0, params.get('m22', 1.0)]])
        M_rot = R_mat @ M_diag @ R_mat.T

        u_disp_x, u_disp_y = 0.0, 0.0
        
        for nid, n_pos in neighbors.items():
            dx = robot_state['xB'] - n_pos['x']
            dy = robot_state['yB'] - n_pos['y']
            dist_sq = max(dx**2 + dy**2, 0.0001)
            
            # Raw dispersion vector (NO beta multiplier here!)
            u_disp_x += dx / dist_sq
            u_disp_y += dy / dist_sq
            
        disp_vec = np.array([u_disp_x, u_disp_y])
        
        # Multiply the raw vector by the rotated M matrix
        u_disp = M_rot @ disp_vec
        
        # Final control law matching the handwritten note exactly
        u_x = -params['rho'] * (robot_state['xB'] - r_x) + u_disp[0] + r_dot_x
        u_y = -params['rho'] * (robot_state['yB'] - r_y) + u_disp[1] + r_dot_y
        
        return u_x, u_y
        
class T1PointMassFlocking(BaseControlLaw):
    """
    T1 Strategy: Point-Mass Flocking.
    Uses y-axis proportional alignment and x-axis point-mass repulsion.
    Warning: Repulsion denominator (x_i - x_j) assumes zero physical volume.
    """
    def compute(self, robot_state, neighbors, traj_ref, params):
        x_i = robot_state['xB']
        y_i = robot_state['yB']
        
        # New parameters specific to this strategy
        K = params.get('K', 2.0)          # Y-axis alignment stiffness
        y_bar = params.get('y_bar', 0.0)  # Target Y consensus line
        rho = params.get('rho', 1.0)      # X-axis goal attraction
        alpha = params.get('alpha', 1.0)  # X-axis swarm cohesion
        beta = params.get('beta', 1.0)    # X-axis point-mass repulsion
        
        # --- Eq 1: Y-Axis Alignment ---
        u_y = -K * (y_i - y_bar)
        
        # --- Eq 2: X-Axis Point-Mass Flocking ---
        u_x = -rho * x_i
        
        for nid, n_pos in neighbors.items():
            x_j = n_pos['x']
            dx = x_i - x_j
            
            # Safeguard: Prevent division by zero if centers perfectly align
            if abs(dx) < 0.0001:
                dx = 0.0001 if dx >= 0 else -0.0001
                
            u_x += -alpha * dx + beta * (1.0 / dx)
            
        return u_x, u_y


class T2DimensionAwareFlocking(BaseControlLaw):
    """
    T2 Strategy: Dimension-Aware Flocking (Safe Regions).
    Uses y-axis proportional alignment and x-axis barrier-certificate repulsion.
    Accounts for the physical volume (d_i + d_j) of the robots.
    """
    def compute(self, robot_state, neighbors, traj_ref, params):
        x_i = robot_state['xB']
        y_i = robot_state['yB']
        
        # Parameters
        K = params.get('K', 2.0)          
        y_bar = params.get('y_bar', 0.0)  
        rho = params.get('rho', 1.0)      
        alpha = params.get('alpha', 1.0)  
        beta = params.get('beta', 1.0)    
        
        # Physical safe region (d_i + d_j). For a TurtleBot Burger, 
        # radius is ~0.089m, so two robots touching is ~0.178m. 
        # Add a buffer for sensor noise.
        safe_dist = params.get('safe_dist', 0.25) 
        
        # --- Eq 1: Y-Axis Alignment ---
        u_y = -K * (y_i - y_bar)
        
        # --- Eq 3: X-Axis Dimension-Aware Flocking ---
        u_x = -rho * x_i
        
        for nid, n_pos in neighbors.items():
            x_j = n_pos['x']
            dx = x_i - x_j
            
            # 1. Calculate direction (sign)
            # If dx is exactly 0, force a positive direction to prevent np.sign(0) from 
            # killing the repulsion force when they are perfectly overlapping.
            if dx == 0.0:
                sign_dx = 1.0
            else:
                sign_dx = np.sign(dx)
                
            # 2. Calculate pure physical clearance
            clearance = abs(dx) - safe_dist
            
            # 3. Safeguard: The Barrier Certificate
            # If clearance goes negative (physical crash), the math will accidentally 
            # pull them together. We must floor the clearance to a tiny positive number
            # so the repulsive force spikes to infinity, acting as a solid wall.
            if clearance <= 0.001:
                clearance = 0.001
                
            u_x += -alpha * dx + beta * (sign_dx / clearance)
            
        return u_x, u_y