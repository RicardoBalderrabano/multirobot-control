"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import re
import os
import itertools

def plot_all_perspectives(csv_files=['tb1_fleet_tracking_results.csv', 
                                     'tb2_fleet_tracking_results.csv', 
                                     'tb3_fleet_tracking_results.csv',
                                     'tb7_fleet_tracking_results.csv']):
    
    BURGER_RADIUS = 0.178 / 2  # 178mm diameter Burger footprint
    colors = ['red', 'blue', 'orange', 'purple', 'cyan']

    def draw_burger_footprint(ax, x, y, color, label_text, is_observer=False):
  
        style = '--' if is_observer else '-'
        circle = plt.Circle((x, y), BURGER_RADIUS, color=color, fill=False, 
                            linestyle=style, linewidth=1.5, alpha=0.8)
        ax.add_patch(circle)
        ax.plot(x, y, marker='.', color=color, markersize=4)
        ax.text(x, y + 0.12, label_text, color=color, fontsize=10, 
                fontweight='bold', ha='center', va='bottom')

    # Create the 4 main figures with 1x4 subplots
    fig1, axes1 = plt.subplots(1, 4, figsize=(32, 8))
    fig1.canvas.manager.set_window_title('Lab Floor Views')
    fig1.suptitle('Lab Floor View (Fixed $3\text{m} \\times 3\text{m}$ Quadrant) - All Perspectives', fontsize=16, fontweight='bold')

    fig2, axes2 = plt.subplots(1, 4, figsize=(32, 8))
    fig2.canvas.manager.set_window_title('Detailed Trajectories')
    fig2.suptitle('Detailed Trajectory Comparison - All Perspectives', fontsize=16, fontweight='bold')

    fig3, axes3 = plt.subplots(1, 4, figsize=(32, 6))
    fig3.canvas.manager.set_window_title('RMSE Analysis')
    fig3.suptitle('RMSE Tracking over Time - All Perspectives', fontsize=16, fontweight='bold')

    fig4, axes4 = plt.subplots(1, 4, figsize=(32, 6))
    fig4.canvas.manager.set_window_title('Inter-Agent Distances')
    fig4.suptitle('Inter-Agent Distance (Consensus Proof) - All Perspectives', fontsize=16, fontweight='bold')

    for idx, file in enumerate(csv_files):
        ax1 = axes1[idx]
        ax2 = axes2[idx]
        ax3 = axes3[idx]
        ax4 = axes4[idx]

        if not os.path.exists(file):
            for ax in [ax1, ax2, ax3, ax4]:
                ax.set_title(f"Missing File: {file}", color='red')
            continue

        df = pd.read_csv(file)
        if df.empty:
            continue

        # Dynamically identify the Observer
        observer_id = df.columns[1].split('_')[0] 
        obs_x_col = f'{observer_id}_gt_x'
        obs_y_col = f'{observer_id}_gt_y'
        
        target_ids = sorted(list(set([re.match(r'(tb\d+)_kf_x', col).group(1) 
                                     for col in df.columns if '_kf_x' in col])))
        
        df['timestamp'] = df['timestamp'] - df['timestamp'].iloc[0]

        # ==========================================
        # 1. LAB FLOOR VIEW (ax1)
        # ==========================================
        if obs_x_col in df.columns:
            ax1.plot(df[obs_x_col], df[obs_y_col], 'k--', alpha=0.3, label=f'{observer_id.upper()} (Observer) GT')
            draw_burger_footprint(ax1, df[obs_x_col].iloc[0], df[obs_y_col].iloc[0], 'black', f'{observer_id.upper()} Start', True)
            draw_burger_footprint(ax1, df[obs_x_col].iloc[-1], df[obs_y_col].iloc[-1], 'black', f'{observer_id.upper()} End', True)

        for i, rid in enumerate(target_ids):
            c = colors[i % len(colors)]
            ax1.plot(df[f'{rid}_gt_x'], df[f'{rid}_gt_y'], color='green', linewidth=1, alpha=0.4, label=f'{rid.upper()} GT' if i==0 else "")
            ax1.plot(df[f'{rid}_kf_x'], df[f'{rid}_kf_y'], color=c, linestyle=':', linewidth=2, label=f'{rid.upper()} Fused EKF')
            
            draw_burger_footprint(ax1, df[f'{rid}_kf_x'].iloc[0], df[f'{rid}_kf_y'].iloc[0], c, f'{rid.upper()} Start')
            draw_burger_footprint(ax1, df[f'{rid}_kf_x'].iloc[-1], df[f'{rid}_kf_y'].iloc[-1], c, f'{rid.upper()} End')

        ax1.set_title(f'Perspective: {observer_id.upper()}', fontsize=14)
        ax1.set_xlim([-3, 3])
        ax1.set_ylim([-3, 3])
        ax1.set_aspect('equal')
        ax1.grid(True, which='both', linestyle='--', alpha=0.5)
        ax1.axhline(0, color='black', linewidth=0.5)
        ax1.axvline(0, color='black', linewidth=0.5)
        ax1.legend(loc='upper right', fontsize='small')

        # ==========================================
        # 2. TRAJECTORY DETAIL (ax2)
        # ==========================================
        if obs_x_col in df.columns:
            ax2.plot(df[obs_x_col], df[obs_y_col], 'k--', alpha=0.2, label=f'{observer_id.upper()} Observer')
        
        performance_stats = []
        for i, rid in enumerate(target_ids):
            c = colors[i % len(colors)]
            ax2.plot(df[f'{rid}_gt_x'], df[f'{rid}_gt_y'], 'g-', alpha=0.3)
            ax2.plot(df[f'{rid}_kf_x'], df[f'{rid}_kf_y'], color=c, linestyle=':', label=f'{rid.upper()} EKF')
            
            # Calculate Errors
            err = np.sqrt((df[f'{rid}_kf_x'] - df[f'{rid}_gt_x'])**2 + (df[f'{rid}_kf_y'] - df[f'{rid}_gt_y'])**2)
            final_rmse = np.sqrt(np.mean(err**2))
            peak_err = np.max(err)
            
            performance_stats.append(f"{rid.upper()} RMSE: {final_rmse:.4f}m | Peak: {peak_err:.4f}m")

        ax2.set_title(f'Details: {observer_id.upper()}', fontsize=14)
        ax2.set_aspect('equal')
        ax2.grid(True)
        ax2.legend()
        
        ax2.text(0.05, 0.05, "PERFORMANCE SUMMARY:\n" + "\n".join(performance_stats), 
                 transform=ax2.transAxes, bbox=dict(facecolor='white', alpha=0.8), fontsize=10)

        # ==========================================
        # 3. RMSE PLOT (ax3)
        # ==========================================
        for i, rid in enumerate(target_ids):
            err = np.sqrt((df[f'{rid}_kf_x'] - df[f'{rid}_gt_x'])**2 + (df[f'{rid}_kf_y'] - df[f'{rid}_gt_y'])**2)
            running_rmse = np.sqrt(np.cumsum(err**2) / (np.arange(len(err)) + 1))
            ax3.plot(df['timestamp'], running_rmse, color=colors[i % len(colors)], label=f'RMSE: {rid.upper()}')
        
        ax3.set_title(f'RMSE: {observer_id.upper()}', fontsize=14)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('RMSE (m)')
        ax3.grid(True)
        ax3.legend()

        # ==========================================
        # 4. INTER-AGENT DISTANCE (ax4)
        # ==========================================
        # Gather all agents (Observer GT + Targets EKF) to calculate pairwise distances
        agents = []
        if obs_x_col in df.columns:
            agents.append({'id': observer_id, 'x': df[obs_x_col], 'y': df[obs_y_col]})
        for rid in target_ids:
            agents.append({'id': rid, 'x': df[f'{rid}_kf_x'], 'y': df[f'{rid}_kf_y']})
        
        color_idx = 0
        for pair in itertools.combinations(agents, 2):
            dist = np.sqrt((pair[0]['x'] - pair[1]['x'])**2 + (pair[0]['y'] - pair[1]['y'])**2)
            label = f"{pair[0]['id'].upper()} to {pair[1]['id'].upper()}"
            ax4.plot(df['timestamp'], dist, color=colors[color_idx % len(colors)], linewidth=2, label=label)
            color_idx += 1
            
        # Draw the target 0.8m formation line
        ax4.axhline(0.5, color='black', linestyle='--', linewidth=2, label='Target Equilibrium (0.8m)')
        
        ax4.set_title(f'Consensus Distances: {observer_id.upper()}', fontsize=14)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Distance (m)')
        # Add some headroom so the 0.8 line is clearly visible
        ax4.set_ylim([0, max(ax4.get_ylim()[1], 1.0)]) 
        ax4.grid(True)
        ax4.legend(loc='lower right')

    # Clean up the layout
    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    fig4.tight_layout()
    
    plt.show()

if __name__ == "__main__":
    plot_all_perspectives()

"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import re
import os
import itertools

def plot_all_perspectives(csv_files=['tb1_fleet_tracking_results.csv', 
                                     'tb2_fleet_tracking_results.csv', 
                                     'tb3_fleet_tracking_results.csv'
                                     #'tb7_fleet_tracking_results.csv'
                                     ]):
    
    BURGER_RADIUS = 0.178 / 2  # 178mm diameter Burger footprint
    colors = ['red', 'blue', 'orange', 'purple', 'cyan']

    def draw_burger_footprint(ax, x, y, color, label_text, is_observer=False):
        """ Draws the circular footprint of a TurtleBot3 Burger """
        style = '--' if is_observer else '-'
        circle = plt.Circle((x, y), BURGER_RADIUS, color=color, fill=False, 
                            linestyle=style, linewidth=1.5, alpha=0.8)
        ax.add_patch(circle)
        ax.plot(x, y, marker='.', color=color, markersize=4)
        ax.text(x, y + 0.12, label_text, color=color, fontsize=10, 
                fontweight='bold', ha='center', va='bottom')

    # Create the 5 main figures with 1x4 subplots (optimized for 4 robots)
    fig1, axes1 = plt.subplots(1, 3, figsize=(32, 8))
    fig1.canvas.manager.set_window_title('Lab Floor Views')
    fig1.suptitle('Lab Floor View (Fixed $3\text{m} \\times 3\text{m}$ Quadrant) - All Perspectives', fontsize=16, fontweight='bold')

    fig2, axes2 = plt.subplots(1, 3, figsize=(32, 8))
    fig2.canvas.manager.set_window_title('Detailed Trajectories')
    fig2.suptitle('Detailed Trajectory Comparison - All Perspectives', fontsize=16, fontweight='bold')

    fig3, axes3 = plt.subplots(1, 3, figsize=(32, 6))
    fig3.canvas.manager.set_window_title('RMSE Analysis')
    fig3.suptitle('RMSE Tracking over Time - All Perspectives', fontsize=16, fontweight='bold')

    fig4, axes4 = plt.subplots(1, 3, figsize=(32, 6))
    fig4.canvas.manager.set_window_title('Inter-Agent Distances (EKF)')
    fig4.suptitle('Inter-Agent Distance (EKF Estimated) - All Perspectives', fontsize=16, fontweight='bold')

    fig5, axes5 = plt.subplots(1, 3, figsize=(32, 6))
    fig5.canvas.manager.set_window_title('Inter-Agent Distances (GT)')
    fig5.suptitle('Inter-Agent Distance (OptiTrack Ground Truth) - All Perspectives', fontsize=16, fontweight='bold')

    for idx, file in enumerate(csv_files):
        ax1 = axes1[idx]
        ax2 = axes2[idx]
        ax3 = axes3[idx]
        ax4 = axes4[idx]
        ax5 = axes5[idx]

        if not os.path.exists(file):
            for ax in [ax1, ax2, ax3, ax4, ax5]:
                ax.set_title(f"Missing File: {file}", color='red')
            continue

        df = pd.read_csv(file)
        if df.empty:
            continue

        # Dynamically identify the Observer
        observer_id = df.columns[1].split('_')[0] 
        obs_x_col = f'{observer_id}_gt_x'
        obs_y_col = f'{observer_id}_gt_y'
        
        target_ids = sorted(list(set([re.match(r'(tb\d+)_kf_x', col).group(1) 
                                     for col in df.columns if '_kf_x' in col])))
        
        df['timestamp'] = df['timestamp'] - df['timestamp'].iloc[0]

        # ==========================================
        # 1. LAB FLOOR VIEW (ax1)
        # ==========================================
        if obs_x_col in df.columns:
            ax1.plot(df[obs_x_col], df[obs_y_col], 'k--', alpha=0.3, label=f'{observer_id.upper()} (Observer) GT')
            draw_burger_footprint(ax1, df[obs_x_col].iloc[0], df[obs_y_col].iloc[0], 'black', f'{observer_id.upper()} Start', True)
            draw_burger_footprint(ax1, df[obs_x_col].iloc[-1], df[obs_y_col].iloc[-1], 'black', f'{observer_id.upper()} End', True)

        for i, rid in enumerate(target_ids):
            c = colors[i % len(colors)]
            ax1.plot(df[f'{rid}_gt_x'], df[f'{rid}_gt_y'], color='green', linewidth=1, alpha=0.4, label=f'{rid.upper()} GT' if i==0 else "")
            ax1.plot(df[f'{rid}_kf_x'], df[f'{rid}_kf_y'], color=c, linestyle=':', linewidth=2, label=f'{rid.upper()} Fused EKF')
            
            draw_burger_footprint(ax1, df[f'{rid}_kf_x'].iloc[0], df[f'{rid}_kf_y'].iloc[0], c, f'{rid.upper()} Start')
            draw_burger_footprint(ax1, df[f'{rid}_kf_x'].iloc[-1], df[f'{rid}_kf_y'].iloc[-1], c, f'{rid.upper()} End')

        ax1.set_title(f'Perspective: {observer_id.upper()}', fontsize=14)
        ax1.set_xlim([-3, 3])
        ax1.set_ylim([-3, 3])
        ax1.set_aspect('equal')
        ax1.grid(True, which='both', linestyle='--', alpha=0.5)
        ax1.axhline(0, color='black', linewidth=0.5)
        ax1.axvline(0, color='black', linewidth=0.5)
        ax1.legend(loc='upper right', fontsize='small')

        # ==========================================
        # 2. TRAJECTORY DETAIL (ax2)
        # ==========================================
        if obs_x_col in df.columns:
            ax2.plot(df[obs_x_col], df[obs_y_col], 'k--', alpha=0.2, label=f'{observer_id.upper()} Observer')
        
        performance_stats = []
        for i, rid in enumerate(target_ids):
            c = colors[i % len(colors)]
            ax2.plot(df[f'{rid}_gt_x'], df[f'{rid}_gt_y'], 'g-', alpha=0.3)
            ax2.plot(df[f'{rid}_kf_x'], df[f'{rid}_kf_y'], color=c, linestyle=':', label=f'{rid.upper()} EKF')
            
            # Calculate Errors
            err = np.sqrt((df[f'{rid}_kf_x'] - df[f'{rid}_gt_x'])**2 + (df[f'{rid}_kf_y'] - df[f'{rid}_gt_y'])**2)
            final_rmse = np.sqrt(np.mean(err**2))
            peak_err = np.max(err)
            
            performance_stats.append(f"{rid.upper()} RMSE: {final_rmse:.4f}m | Peak: {peak_err:.4f}m")

        ax2.set_title(f'Details: {observer_id.upper()}', fontsize=14)
        ax2.set_aspect('equal')
        ax2.grid(True)
        ax2.legend()
        
        ax2.text(0.05, 0.05, "PERFORMANCE SUMMARY:\n" + "\n".join(performance_stats), 
                 transform=ax2.transAxes, bbox=dict(facecolor='white', alpha=0.8), fontsize=10)

        # ==========================================
        # 3. RMSE PLOT (ax3)
        # ==========================================
        for i, rid in enumerate(target_ids):
            err = np.sqrt((df[f'{rid}_kf_x'] - df[f'{rid}_gt_x'])**2 + (df[f'{rid}_kf_y'] - df[f'{rid}_gt_y'])**2)
            running_rmse = np.sqrt(np.cumsum(err**2) / (np.arange(len(err)) + 1))
            ax3.plot(df['timestamp'], running_rmse, color=colors[i % len(colors)], label=f'RMSE: {rid.upper()}')
        
        ax3.set_title(f'RMSE: {observer_id.upper()}', fontsize=14)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('RMSE (m)')
        ax3.grid(True)
        ax3.legend()

        # ==========================================
        # 4. INTER-AGENT DISTANCE - EKF (ax4)
        # ==========================================
        agents_kf = []
        if obs_x_col in df.columns:
            agents_kf.append({'id': observer_id, 'x': df[obs_x_col], 'y': df[obs_y_col]})
        for rid in target_ids:
            agents_kf.append({'id': rid, 'x': df[f'{rid}_kf_x'], 'y': df[f'{rid}_kf_y']})
        
        color_idx = 0
        for pair in itertools.combinations(agents_kf, 2):
            dist = np.sqrt((pair[0]['x'] - pair[1]['x'])**2 + (pair[0]['y'] - pair[1]['y'])**2)
            label = f"{pair[0]['id'].upper()} to {pair[1]['id'].upper()}"
            ax4.plot(df['timestamp'], dist, color=colors[color_idx % len(colors)], linewidth=2, label=label)
            color_idx += 1
            
        ax4.set_title(f'EKF Distances: {observer_id.upper()}', fontsize=14)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Distance (m)')
        ax4.set_ylim([0, max(ax4.get_ylim()[1], 1.0)]) 
        ax4.grid(True)
        ax4.legend(loc='lower right')

        # ==========================================
        # 5. INTER-AGENT DISTANCE - GROUND TRUTH (ax5)
        # ==========================================
        agents_gt = []
        if obs_x_col in df.columns:
            agents_gt.append({'id': observer_id, 'x': df[obs_x_col], 'y': df[obs_y_col]})
        for rid in target_ids:
            # Ensure the GT columns exist for the targets
            if f'{rid}_gt_x' in df.columns and f'{rid}_gt_y' in df.columns:
                agents_gt.append({'id': rid, 'x': df[f'{rid}_gt_x'], 'y': df[f'{rid}_gt_y']})
        
        color_idx = 0
        for pair in itertools.combinations(agents_gt, 2):
            dist = np.sqrt((pair[0]['x'] - pair[1]['x'])**2 + (pair[0]['y'] - pair[1]['y'])**2)
            label = f"{pair[0]['id'].upper()} to {pair[1]['id'].upper()}"
            ax5.plot(df['timestamp'], dist, color=colors[color_idx % len(colors)], linewidth=2, label=label)
            color_idx += 1
            
        ax5.set_title(f'GT Distances: {observer_id.upper()}', fontsize=14)
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Distance (m)')
        ax5.set_ylim([0, max(ax5.get_ylim()[1], 1.0)]) 
        ax5.grid(True)
        ax5.legend(loc='lower right')

    # Clean up the layout
    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    fig4.tight_layout()
    fig5.tight_layout()
    
    plt.show()

if __name__ == "__main__":
    plot_all_perspectives()