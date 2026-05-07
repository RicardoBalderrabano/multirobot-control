#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import itertools
import os
import sys

# ==========================================
# CONFIGURATION
# ==========================================
SAVE_DIR = '/home/ricardo/multirobot_ws/src/multirobot_control/turtlebot_simulation_inout_linearization/plotting'
MODE_NAME = 'tangent_sine' # Updated to match your current test
CSV_FILENAME = f'swarm_effort_{MODE_NAME}.csv'
CSV_PATH = os.path.join(SAVE_DIR, CSV_FILENAME)

BURGER_RADIUS = 0.178 / 2  # 178mm diameter Burger footprint
COLORS = ['#d62728', '#1f77b4', '#ff7f0e', '#2ca02c', '#9467bd', '#8c564b']

if not os.path.exists(CSV_PATH):
    print(f"❌ Error: Could not find {CSV_FILENAME} in {SAVE_DIR}")
    sys.exit(1)

# Load data
df = pd.read_csv(CSV_PATH)
if 'x' not in df.columns or 'y' not in df.columns:
    print("❌ Error: CSV missing 'x' and 'y' data.")
    sys.exit(1)

# ==========================================
# 🛠️ AXIS FIX (SWAP OR INVERT)
# ==========================================
# Toggle these to True/False to perfectly match your lab's visual orientation
SWAP_X_AND_Y = False  # Swaps X and Y axes 
INVERT_X = False     # Flips left/right
INVERT_Y = False     # Flips up/down

if SWAP_X_AND_Y:
    df_copy = df.copy()
    df['x'] = df_copy['y']
    df['y'] = df_copy['x']
    
if INVERT_X:
    df['x'] = -df['x']
if INVERT_Y:
    df['y'] = -df['y']
# ==========================================

# Normalize time to start at 0
df['time_norm'] = df['timestamp'] - df['timestamp'].min()
robots = sorted(df['robot_id'].unique())
color_map = {rid: COLORS[i % len(COLORS)] for i, rid in enumerate(robots)}

def draw_burger_footprint(ax, x, y, color, label_text):
    """ Draws the circular footprint of a TurtleBot3 Burger """
    circle = plt.Circle((x, y), BURGER_RADIUS, color=color, fill=False, linestyle='-', linewidth=2, alpha=0.8)
    ax.add_patch(circle)
    ax.plot(x, y, marker='.', color=color, markersize=6)
    ax.text(x, y + 0.12, label_text, color=color, fontsize=10, fontweight='bold', ha='center', va='bottom')

# ==========================================
# PLOT 1: LAB FLOOR VIEW (3m x 3m Quadrant)
# ==========================================
fig1, ax1 = plt.subplots(figsize=(10, 10))
fig1.canvas.manager.set_window_title('Simulation Floor View')
fig1.suptitle(f'Simulation Lab Floor View ({MODE_NAME.replace("_", " ").title()})', fontsize=16, fontweight='bold')

for rid in robots:
    r_df = df[df['robot_id'] == rid]
    
    # Draw Trajectory
    ax1.plot(r_df['x'], r_df['y'], color=color_map[rid], linewidth=2.5, label=f'{rid.upper()} Ground Truth')
    
    # Draw Start and End footprints
    draw_burger_footprint(ax1, r_df['x'].iloc[0], r_df['y'].iloc[0], 'black', f'{rid.upper()} Start')
    draw_burger_footprint(ax1, r_df['x'].iloc[-1], r_df['y'].iloc[-1], color_map[rid], f'{rid.upper()} End')

# Format Quadrant Grid
ax1.set_xlim([-3, 3])
ax1.set_ylim([-3, 3])
ax1.set_aspect('equal')
ax1.grid(True, which='both', linestyle='--', alpha=0.6)
ax1.axhline(0, color='black', linewidth=1)
ax1.axvline(0, color='black', linewidth=1)
ax1.set_xlabel('X Coordinate (m)', fontsize=12)
ax1.set_ylabel('Y Coordinate (m)', fontsize=12)
ax1.legend(loc='upper right', fontsize=10)

# Save Image
fig1_path = os.path.join(SAVE_DIR, f'floor_view_{MODE_NAME}.png')
fig1.savefig(fig1_path, dpi=300, bbox_inches='tight')

# ==========================================
# PLOT 2: INTER-AGENT DISTANCES (Subplot per Robot)
# ==========================================
# Align asynchronous timestamps
df['time_bin'] = df['time_norm'].round(1)
pivot_x = df.pivot_table(index='time_bin', columns='robot_id', values='x').ffill().dropna()
pivot_y = df.pivot_table(index='time_bin', columns='robot_id', values='y').ffill().dropna()

num_robots = len(robots)
# Create a 1xN grid of subplots (e.g., 1x4 if 4 robots)
fig2, axes2 = plt.subplots(1, num_robots, figsize=(6 * num_robots, 6))
if num_robots == 1:
    axes2 = [axes2] # Handle edge case

fig2.canvas.manager.set_window_title('Inter-Agent Distances')
fig2.suptitle(f'Inter-Agent Distance (Consensus Proof) - All Perspectives', fontsize=16, fontweight='bold')

# Loop through each robot to act as the "Observer" for its own subplot
for idx, obs_id in enumerate(robots):
    ax = axes2[idx]
    color_idx = 0
    
    for target_id in robots:
        if obs_id == target_id:
            continue # Don't calculate distance to itself
            
        if obs_id in pivot_x.columns and target_id in pivot_x.columns:
            dx = pivot_x[obs_id] - pivot_x[target_id]
            dy = pivot_y[obs_id] - pivot_y[target_id]
            dist = np.sqrt(dx**2 + dy**2)
            
            label = f"{obs_id.upper()} to {target_id.upper()}"
            # Using red, blue, orange style colors
            ax.plot(pivot_x.index, dist, color=COLORS[color_idx % len(COLORS)], linewidth=2, label=label)
            color_idx += 1

    # Format each individual subplot to match your real-world graphs
    ax.set_title(f'Consensus Distances: {obs_id.upper()}', fontsize=14)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Distance (m)', fontsize=12)
    
    # Set Y-axis to start at 0, dynamically scale top, add solid grid
    ax.set_ylim(ymin=0, ymax=1.3) 
    ax.grid(True, linestyle='-', alpha=0.7)
    ax.legend(loc='lower right', fontsize=10)

fig2.tight_layout()

# Save Image
fig2_path = os.path.join(SAVE_DIR, f'distances_{MODE_NAME}.png')
fig2.savefig(fig2_path, dpi=300, bbox_inches='tight')

print(f"✅ Graphs saved to: {SAVE_DIR}")

# Show interactive windows
plt.show()