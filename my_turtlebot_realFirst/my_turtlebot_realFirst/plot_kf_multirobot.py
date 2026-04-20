# PLOT ONLY ONE PERSPECTIVE 
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import re

def plot_fleet_results(csv_file='tb2_fleet_tracking_results.csv'):
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: {csv_file} not found.")
        return

    # Constants
    BURGER_RADIUS = 0.178 / 2  # 178mm diameter Burger footprint
    
    # --- THE FIX: Dynamically identify the Observer ---
    # The EKF always puts the observer's ground truth in the 2nd column
    observer_id = df.columns[1].split('_')[0] 
    obs_x_col = f'{observer_id}_gt_x'
    obs_y_col = f'{observer_id}_gt_y'
    
    # Identify target robots (tb1, tb2, tb3, etc.)
    target_ids = sorted(list(set([re.match(r'(tb\d+)_kf_x', col).group(1) 
                                 for col in df.columns if '_kf_x' in col])))
    
    df['timestamp'] = df['timestamp'] - df['timestamp'].iloc[0]
    colors = ['red', 'blue', 'orange', 'purple', 'cyan']

    def draw_burger_footprint(ax, x, y, color, label_text, is_observer=False):
        """ Draws the circular footprint of a TurtleBot3 Burger """
        style = '--' if is_observer else '-'
        circle = plt.Circle((x, y), BURGER_RADIUS, color=color, fill=False, 
                            linestyle=style, linewidth=1.5, alpha=0.8)
        ax.add_patch(circle)
        ax.plot(x, y, marker='.', color=color, markersize=4)
        ax.text(x, y + 0.12, label_text, color=color, fontsize=8, 
                fontweight='bold', ha='center', va='bottom')

    # --- FIGURE 1: THE LAB FLOOR VIEW (Fixed Quadrant) ---
    fig1, ax1 = plt.subplots(figsize=(10, 10))
    
    # 1. Plot Observer (Dynamic)
    if obs_x_col in df.columns:
        ax1.plot(df[obs_x_col], df[obs_y_col], 'k--', alpha=0.3, label=f'{observer_id.upper()} (Observer) GT')
        draw_burger_footprint(ax1, df[obs_x_col].iloc[0], df[obs_y_col].iloc[0], 'black', f'{observer_id.upper()} Start', True)
        draw_burger_footprint(ax1, df[obs_x_col].iloc[-1], df[obs_y_col].iloc[-1], 'black', f'{observer_id.upper()} End', True)

    # 2. Plot Targets
    for i, rid in enumerate(target_ids):
        c = colors[i % len(colors)]
        ax1.plot(df[f'{rid}_gt_x'], df[f'{rid}_gt_y'], color='green', linewidth=1, alpha=0.4, label=f'{rid.upper()} GT' if i==0 else "")
        ax1.plot(df[f'{rid}_kf_x'], df[f'{rid}_kf_y'], color=c, linestyle=':', linewidth=2, label=f'{rid.upper()} Fused KF')
        
        # Start/End circles
        draw_burger_footprint(ax1, df[f'{rid}_kf_x'].iloc[0], df[f'{rid}_kf_y'].iloc[0], c, f'{rid.upper()} Start')
        draw_burger_footprint(ax1, df[f'{rid}_kf_x'].iloc[-1], df[f'{rid}_kf_y'].iloc[-1], c, f'{rid.upper()} End')

    ax1.set_title(f'{observer_id.upper()} LAB FLOOR VIEW (Fixed $2\text{{m}} \\times 2\text{{m}}$ Quadrant)')
    ax1.set_xlim([-2, 2])
    ax1.set_ylim([-2, 2])
    ax1.set_aspect('equal')
    ax1.grid(True, which='both', linestyle='--', alpha=0.5)
    ax1.axhline(0, color='black', linewidth=0.5)
    ax1.axvline(0, color='black', linewidth=0.5)
    ax1.legend(loc='upper right', fontsize='small')

    # --- FIGURE 2: TRAJECTORY DETAIL (Auto-Scaled) ---
    fig2, ax2 = plt.subplots(figsize=(10, 8))
    
    if obs_x_col in df.columns:
        ax2.plot(df[obs_x_col], df[obs_y_col], 'k--', alpha=0.2, label=f'{observer_id.upper()} Observer')
    
    performance_stats = []
    for i, rid in enumerate(target_ids):
        c = colors[i % len(colors)]
        ax2.plot(df[f'{rid}_gt_x'], df[f'{rid}_gt_y'], 'g-', alpha=0.3)
        ax2.plot(df[f'{rid}_kf_x'], df[f'{rid}_kf_y'], color=c, linestyle=':', label=f'{rid.upper()} KF')
        
        err = np.sqrt((df[f'{rid}_kf_x'] - df[f'{rid}_gt_x'])**2 + (df[f'{rid}_kf_y'] - df[f'{rid}_gt_y'])**2)
        performance_stats.append(f"{rid.upper()} Final RMSE: {np.sqrt(np.mean(err**2)):.4f} m")

    ax2.set_title('Detailed Trajectory Comparison')
    ax2.set_aspect('equal')
    ax2.grid(True)
    ax2.legend()
    
    plt.gcf().text(0.15, 0.02, "PERFORMANCE SUMMARY:\n" + "\n".join(performance_stats), 
                   bbox=dict(facecolor='white', alpha=0.8))

    # --- FIGURE 3: RMSE PER ROBOT ---
    plt.figure(figsize=(10, 5))
    for i, rid in enumerate(target_ids):
        err = np.sqrt((df[f'{rid}_kf_x'] - df[f'{rid}_gt_x'])**2 + (df[f'{rid}_kf_y'] - df[f'{rid}_gt_y'])**2)
        running_rmse = np.sqrt(np.cumsum(err**2) / (np.arange(len(err)) + 1))
        plt.plot(df['timestamp'], running_rmse, color=colors[i % len(colors)], label=f'RMSE: {rid.upper()}')
    
    plt.title('Accuracy Tracking over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('RMSE (m)')
    plt.grid(True)
    plt.legend()

    plt.show()

if __name__ == "__main__":
    # Change this line depending on which robot's brain you want to look at!
    plot_fleet_results('tb1_fleet_tracking_results.csv')