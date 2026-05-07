#!/usr/bin/env python3
"""
control_strategies.py
Library of Distributed Control Laws for Swarm Robotics.
"""

import numpy as np
import math

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
            
        r_x, r_y, r_dot_x, r_dot_y = traj_ref[:4]
        
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

class TangentMappingFlocking(BaseControlLaw):
    """
    Based on Fedele, D'Alfonso, Chen (2025).
    Transforms agents into a virtual tangent frame (F_v), applies finite-time
    convergence to the path (Eq 12), and flocks along the tangent (Eq 11).
    """
    def compute(self, robot_state, neighbors, traj_ref, params):
        if traj_ref is None: return 0.0, 0.0
            
        x_B = robot_state['xB']
        y_B = robot_state['yB']
        
        # Reference trajectory data
        r_x, r_y, r_dot_x, r_dot_y = traj_ref[:4]
        
        # Paper Parameters (Example 1 uses alpha=1.0, rho=1.0, mu=0.4)
        rho = params.get('rho', 1.0)      
        alpha = params.get('alpha', 1.0)  
        beta = params.get('beta', 1.0)    
        mu = params.get('mu', 0.4)        # Fractional power for finite-time convergence
        safe_dist = params.get('safe_dist', 0.30) 
        
        # Get the angular velocity of the reference trajectory
        # For a circle, this is exactly traj_w.
        theta_dot = params.get('traj_w', 0.12)
        
        # --- STEP 1: Compute Diffeomorphism (Real to Virtual Frame) ---
        # 1a. Calculate the angle of the tangent line
        theta = math.atan2(r_dot_y, r_dot_x)
        c_th, s_th = math.cos(theta), math.sin(theta)
        
        # 1b. Real world errors
        e_x = x_B - r_x
        e_y = y_B - r_y
        
        # 1c. Rotate into virtual frame (xi along tangent, zeta perpendicular)
        xi_i = c_th * e_x + s_th * e_y
        zeta_i = -s_th * e_x + c_th * e_y
        
        # Map neighbors into virtual frame
        xi_neighbors = []
        for nid, n_pos in neighbors.items():
            n_ex = n_pos['x'] - r_x
            n_ey = n_pos['y'] - r_y
            xi_j = c_th * n_ex + s_th * n_ey
            xi_neighbors.append(xi_j)
            
        # --- STEP 2: Virtual Control Laws (Modified) ---
        
        # Modified Eq 12: Zeta-axis (Perpendicular to curve) - Proportional Convergence
        K = params.get('K', 2.0)
        
        if abs(zeta_i) < 0.02:
            u_zeta = 0.0
        else:
            u_zeta = -K * zeta_i
            
        # Eq 11: Xi-axis (Along the tangent curve) - Flocking
        u_xi = -rho * xi_i
        
        for xi_j in xi_neighbors:
            dx = xi_i - xi_j
            
            # Hardware Barrier Certificate (Upgraded from pure Eq 11 for safety)
            sign_dx = 1.0 if dx == 0.0 else np.sign(dx)
            clearance = abs(dx) - safe_dist
            if clearance <= 0.001: clearance = 0.001
                
            raw_rep = beta * (sign_dx / clearance)
            clipped_rep = np.clip(raw_rep, -1.0, 1.0)
            
            u_xi += -alpha * dx + clipped_rep
            
        # --- STEP 3: Inverse Diffeomorphism (Algorithm 1, Step 4) ---
        # Maps u_xi and u_zeta back to real X,Y forces, accounting for 
        # the frame rotation (Coriolis effect) and feedforward velocity.
        
        u_x = (c_th * u_xi) - (s_th * u_zeta) - (theta_dot * e_y) + r_dot_x
        u_y = (s_th * u_xi) + (c_th * u_zeta) + (theta_dot * e_x) + r_dot_y
        
        # Final Force Deadband
        if abs(u_x) < 0.05 and abs(u_y) < 0.05:
            return 0.0, 0.0
            
        return u_x, u_y

class DynamicTangentMappingFlocking(BaseControlLaw):
    """
    Based on Fedele, D'Alfonso, Chen (2025).
    Updated for dynamic curvature trajectories (Sine waves) and Group Isolation.
    Transforms agents into a virtual tangent frame (F_v), applies proportional
    convergence to the path (Modified Eq 12), and flocks along the tangent (Modified Eq 11).
    """
    def compute(self, robot_state, neighbors, traj_ref, params):
        # We now expect 5 items: [r_x, r_y, r_dot_x, r_dot_y, theta_dot]
        if traj_ref is None or len(traj_ref) < 5: 
            return 0.0, 0.0
            
        x_B = robot_state['xB']
        y_B = robot_state['yB']
        
        # 1. Unpack the dynamic reference trajectory data
        r_x, r_y, r_dot_x, r_dot_y, theta_dot = traj_ref
        
        # Paper Parameters
        rho = params.get('rho', 1.0)      
        alpha = params.get('alpha', 1.0)  
        beta = params.get('beta', 1.0)    
        safe_dist = params.get('safe_dist', 0.30) 
        
        # Group Isolation Parameter (e.g., ['tb1', 'tb2'])
        valid_group = params.get('group_members', None)
        
        # --- STEP 1: Compute Diffeomorphism (Real to Virtual Frame) ---
        # 1a. Calculate the angle of the tangent line
        theta = math.atan2(r_dot_y, r_dot_x)
        c_th, s_th = math.cos(theta), math.sin(theta)
        
        # 1b. Real world errors
        e_x = x_B - r_x
        e_y = y_B - r_y
        
        # 1c. Rotate into virtual frame (xi along tangent, zeta perpendicular)
        xi_i = c_th * e_x + s_th * e_y
        zeta_i = -s_th * e_x + c_th * e_y
        
        # Map neighbors into virtual frame
        xi_neighbors = []
        for nid, n_pos in neighbors.items():
            
            # --- ISOLATION LOGIC ---
            # If a group is defined, ignore any robot not in the group
            if valid_group is not None and nid not in valid_group:
                continue
                
            n_ex = n_pos['x'] - r_x
            n_ey = n_pos['y'] - r_y
            xi_j = c_th * n_ex + s_th * n_ey
            xi_neighbors.append(xi_j)
            
        # --- STEP 2: Virtual Control Laws (Modified) ---
        
        # Modified Eq 12: Zeta-axis (Perpendicular to curve) - Proportional Convergence
        K = params.get('K', 2.0)
        
        if abs(zeta_i) < 0.02:
            u_zeta = 0.0
        else:
            u_zeta = -K * zeta_i
            
        # Eq 11: Xi-axis (Along the tangent curve) - Flocking
        u_xi = -rho * xi_i
        
        for xi_j in xi_neighbors:
            dx = xi_i - xi_j
            
            # Hardware Barrier Certificate (Upgraded from pure Eq 11 for safety)
            sign_dx = 1.0 if dx == 0.0 else np.sign(dx)
            clearance = abs(dx) - safe_dist
            if clearance <= 0.001: clearance = 0.001
                
            raw_rep = beta * (sign_dx / clearance)
            clipped_rep = np.clip(raw_rep, -1.0, 1.0)
            
            u_xi += -alpha * dx + clipped_rep
            
        # --- STEP 3: Inverse Diffeomorphism (Algorithm 1, Step 4) ---
        # Maps u_xi and u_zeta back to real X,Y forces, accounting for 
        # the frame rotation (Coriolis effect) and feedforward velocity.
        
        u_x = (c_th * u_xi) - (s_th * u_zeta) - (theta_dot * e_y) + r_dot_x
        u_y = (s_th * u_xi) + (c_th * u_zeta) + (theta_dot * e_x) + r_dot_y
        
        # Final Force Deadband
        if abs(u_x) < 0.05 and abs(u_y) < 0.05:
            return 0.0, 0.0
            
        return u_x, u_y