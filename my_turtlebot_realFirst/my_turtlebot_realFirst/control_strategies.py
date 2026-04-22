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
        M_diag = np.array([[params.get('m11', 1.0), 0.0], 
                           [0.0, params.get('m22', 1.0)]])
        M_rot = R_mat @ M_diag @ R_mat.T

        u_disp_x, u_disp_y = 0.0, 0.0
        u_coh_x, u_coh_y = 0.0, 0.0   # NEW: Cohesion terms
        
        for nid, n_pos in neighbors.items():
            dx = robot_state['xB'] - n_pos['x']
            dy = robot_state['yB'] - n_pos['y']
            dist_sq = max(dx**2 + dy**2, 0.0001)
            
            # Anisotropic Repulsion
            u_disp_x += dx / dist_sq
            u_disp_y += dy / dist_sq
            
            # NEW: Isotropic Attraction
            u_coh_x += -params.get('alpha', 1.0) * dx
            u_coh_y += -params.get('alpha', 1.0) * dy
            
        disp_vec = np.array([u_disp_x, u_disp_y])
        u_disp = M_rot @ disp_vec
        
        # Add tracking, feedforward, dispersion, AND cohesion
        u_x = -params['rho'] * (robot_state['xB'] - r_x) + u_disp[0] + u_coh_x + r_dot_x
        u_y = -params['rho'] * (robot_state['yB'] - r_y) + u_disp[1] + u_coh_y + r_dot_y
        
        return u_x, u_y