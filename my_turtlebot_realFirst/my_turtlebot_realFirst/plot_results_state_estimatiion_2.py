"""
================================================================================
PROJECT: Multi-Robot Tracking Analysis
SCRIPT: plot_raw_global_ground_truth.py WORKING WITHOUT TIME STAMP MESSAGES
================================================================================
DESCRIPTION:
    Visualizes the absolute global trajectories of TB1 (Observer) and TB2 (Target)
    based strictly on OptiTrack data.
    - Origin (0,0) is the OptiTrack Arena center.
    - Ignores all estimation/prediction data (Kalman/Odom).
================================================================================


import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pandas as pd
import os

def plot_raw_global(csv_file):
    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} not found!")
        return

    # Load the latest experimental data
    data = pd.read_csv(csv_file)
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Standard TurtleBot3 radius for footprint visualization
    robot_radius = 0.105 
    
    # --- 1. PLOT TB1 (OBSERVER) GLOBAL PATH ---
    ax.plot(data['opti_tb1_x'], data['opti_tb1_y'], color='magenta', 
            linewidth=2, label='TB1 Path (OptiTrack)')
    
    # Add TB1 footprint at START and END to show direction of motion
    tb1_start = patches.Circle((data['opti_tb1_x'].iloc[0], data['opti_tb1_y'].iloc[0]), 
                               robot_radius, color='magenta', alpha=0.1)
    tb1_end = patches.Circle((data['opti_tb1_x'].iloc[-1], data['opti_tb1_y'].iloc[-1]), 
                             robot_radius, color='magenta', alpha=0.3)
    ax.add_patch(tb1_start)
    ax.add_patch(tb1_end)

    # --- 2. PLOT TB2 (TARGET) GLOBAL PATH ---
    # In your experiment, TB2 was static, so this will appear as a single point
    ax.plot(data['opti_tb2_x'], data['opti_tb2_y'], color='black', 
            linewidth=2, label='TB2 Path (OptiTrack)')
    
    # Add TB2 footprint (Static position)
    tb2_footprint = patches.Circle((data['opti_tb2_x'].iloc[0], data['opti_tb2_y'].iloc[0]), 
                                   robot_radius, color='black', alpha=0.2)
    ax.add_patch(tb2_footprint)

    # --- 3. ARENA CONTEXT ---
    # Mark the OptiTrack Arena Origin (0,0)
    ax.axhline(0, color='gray', linestyle='--', alpha=0.3)
    ax.axvline(0, color='gray', linestyle='--', alpha=0.3)
    ax.plot(0, 0, 'kx', markersize=10, label='Arena Origin (0,0)')

    # Formatting the plot
    ax.set_title('Global Ground Truth: TB1 and TB2 (OptiTrack Only)', fontsize=14)
    ax.set_xlabel('Global X Position (meters)', fontsize=12)
    ax.set_ylabel('Global Y Position (meters)', fontsize=12)
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.axis('equal') # Maintain 1:1 aspect ratio for accurate geometry
    
    plt.savefig('raw_global_tracking_results_52.png', dpi=300)
    plt.show()

if __name__ == "__main__":
    plot_raw_global('distributed_tracking_results_52.csv')
"""
"""
================================================================================
PROJECT: Multi-Robot Tracking Analysis
SCRIPT: plot_dynamic_global_analysis.py
================================================================================
DESCRIPTION:
    Visualizes absolute global trajectories using high-precision timestamps.
    - RECONSTRUCTS time from sec + nanosec for accurate kinematic analysis.
    - PLOTS Global Ground Truth paths for TB1 and TB2.
    - CALCULATES inter-robot distance over time to verify experiment consistency.
================================================================================
"""

"""
================================================================================
PROJECT: Multi-Robot Tracking Analysis
SCRIPT: plot_global_and_temporal_tracking.py
================================================================================
DESCRIPTION:
    Processes stamped OptiTrack data to visualize absolute global motion.
    - RECONSTRUCTS high-precision time from nanoseconds.
    - PLOT 1: Global Trajectory (X vs Y) in the arena frame.
    - PLOT 2: X-Position vs Time for both robots.
    - PLOT 3: Y-Position vs Time for both robots.
================================================================================
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pandas as pd
import numpy as np
import os

def plot_stamped_results_cm(csv_file):
    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} not found!")
        return

    # Load experimental data
    data = pd.read_csv(csv_file)
    
    # --- 1. CLEANING DATA ---
    # Filter out initialization rows with sec=0 which squash the plot scale
    data = data[data['sec'] > 0].reset_index(drop=True)
    
    # --- 2. TIME RECONSTRUCTION ---
    # Combine seconds and nanoseconds for high-precision X-axis
    data['total_time'] = data['sec'] + (data['nanosec'] * 1e-9)
    data['time_elapsed'] = data['total_time'] - data['total_time'].iloc[0]

    # --- 3. CONVERT TO CENTIMETERS ---
    # Scaling factor: 1 meter = 100 centimeters
    pos_cols = ['odom_x', 'odom_y', 'lidar_x', 'lidar_y', 'kf_x', 'kf_y', 
                'opti_tb2_x', 'opti_tb2_y', 'opti_tb1_x', 'opti_tb1_y']
    for col in pos_cols:
        data[col] = data[col] * 100

    # Robot radius converted to cm (~10.5 cm)
    robot_radius_cm = 10.5 

    # Initialize subplots: 1 Map and 2 Temporal plots
    fig = plt.figure(figsize=(12, 18))
    gs = fig.add_gridspec(3, 1, height_ratios=[2, 1, 1])
    ax_map = fig.add_subplot(gs[0])
    ax_x_time = fig.add_subplot(gs[1])
    ax_y_time = fig.add_subplot(gs[2])

    # --- PLOT 1: GLOBAL MAP (X vs Y in cm) ---
    ax_map.plot(data['opti_tb1_x'], data['opti_tb1_y'], color='magenta', 
                linewidth=2.5, label='TB1 (Observer) Path')
    ax_map.plot(data['opti_tb2_x'], data['opti_tb2_y'], color='black', 
                linewidth=2.5, label='TB2 (Target) Path')
    
    # Add Footprints at START and END using cm scale
    for robot, color in [('tb1', 'magenta'), ('tb2', 'black')]:
        start_pos = (data[f'opti_{robot}_x'].iloc[0], data[f'opti_{robot}_y'].iloc[0])
        end_pos = (data[f'opti_{robot}_x'].iloc[-1], data[f'opti_{robot}_y'].iloc[-1])
        ax_map.add_patch(patches.Circle(start_pos, robot_radius_cm, color=color, alpha=0.1))
        ax_map.add_patch(patches.Circle(end_pos, robot_radius_cm, color=color, alpha=0.3))

    ax_map.set_title('Global Map: X vs Y Trajectories (Centimeters)', fontsize=14)
    ax_map.set_xlabel('Global X Position (cm)', fontsize=12)
    ax_map.set_ylabel('Global Y Position (cm)', fontsize=12)
    ax_map.legend(loc='best')
    ax_map.grid(True, linestyle=':', alpha=0.6)
    ax_map.axis('equal') # Ensure circles look like circles

    # --- PLOT 2: X-POSITION VS TIME (cm) ---
    ax_x_time.plot(data['time_elapsed'], data['opti_tb1_x'], 'm-', label='TB1 X')
    ax_x_time.plot(data['time_elapsed'], data['opti_tb2_x'], 'k-', label='TB2 X')
    ax_x_time.set_title('Global X-Position over Time (Centimeters)', fontsize=14)
    ax_x_time.set_ylabel('X Position (cm)', fontsize=12)
    ax_x_time.legend(loc='upper right')
    ax_x_time.grid(True, linestyle=':', alpha=0.6)

    # --- PLOT 3: Y-POSITION VS TIME (cm) ---
    ax_y_time.plot(data['time_elapsed'], data['opti_tb1_y'], 'm-', label='TB1 Y')
    ax_y_time.plot(data['time_elapsed'], data['opti_tb2_y'], 'k-', label='TB2 Y')
    ax_y_time.set_title('Global Y-Position over Time (Centimeters)', fontsize=14)
    ax_y_time.set_xlabel('Elapsed Time (seconds)', fontsize=12)
    ax_y_time.set_ylabel('Y Position (cm)', fontsize=12)
    ax_y_time.legend(loc='upper right')
    ax_y_time.grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout()
    plt.savefig('stamped_global_temporal_analysis_cm.png', dpi=300)
    plt.show()

if __name__ == "__main__":
    plot_stamped_results_cm('distributed_tracking_results_stamped.csv')