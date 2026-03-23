"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_benchmarking_results(csv_file):
    try:
        # Load the CSV (Ensure headers: timestamp, kf_x, kf_y, ekf_raw_x, ekf_raw_y, gt_x, gt_y, gt_tb1_x, gt_tb1_y)
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: {csv_file} not found.")
        return

    # Normalize time
    df['timestamp'] = df['timestamp'] - df['timestamp'].iloc[0]

    # Calculate Euclidean Errors
    err_ekf = np.sqrt((df['ekf_raw_x'] - df['gt_x'])**2 + (df['ekf_raw_y'] - df['gt_y'])**2)
    err_kf = np.sqrt((df['kf_x'] - df['gt_x'])**2 + (df['kf_y'] - df['gt_y'])**2)

    # Calculate RMSE (Running RMSE for better visualization over time)
    rmse_ekf_running = np.sqrt(np.cumsum(err_ekf**2) / (np.arange(len(err_ekf)) + 1))
    rmse_kf_running = np.sqrt(np.cumsum(err_kf**2) / (np.arange(len(err_kf)) + 1))

    # Create Figure
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle('Global Multi-Robot State Estimation: Benchmarking', fontsize=20)

    # --- Plot 1: Global Trajectories (The "World" Map) ---
    ax1 = fig.add_subplot(211)
    # Observer Path (Ground Truth)
    ax1.plot(df['gt_tb1_x'], df['gt_tb1_y'], color='black', linestyle='--', label='Observer (TB1) Ground Truth', alpha=0.4)
    # Target Path (Ground Truth)
    ax1.plot(df['gt_x'], df['gt_y'], color='green', label='Target (TB3) Ground Truth', linewidth=3)
    # Target Path (Raw EKF)
    ax1.plot(df['ekf_raw_x'], df['ekf_raw_y'], color='blue', linestyle='--', label='TB3 Raw EKF (Odom Only)', alpha=0.7)
    # Target Path (Fused KF)
    ax1.plot(df['kf_x'], df['kf_y'], color='red', linestyle=':', label='TB3 Fused KF (Lidar + Odom)', linewidth=2)
    
    ax1.set_title('Global Path Comparison (OptiTrack World Frame)', fontsize=14)
    ax1.set_xlabel('Global X [m]')
    ax1.set_ylabel('Global Y [m]')
    ax1.legend(loc='upper right')
    ax1.grid(True)

    # --- Plot 2: RMSE Comparison (Accuracy Benchmarking) ---
    ax2 = fig.add_subplot(212)
    ax2.plot(df['timestamp'], rmse_ekf_running, color='blue', label='Running RMSE: Raw EKF', alpha=0.6)
    ax2.plot(df['timestamp'], rmse_kf_running, color='red', label='Running RMSE: Fused KF', linewidth=2)
    
    ax2.set_title('Root Mean Square Error (RMSE) Analysis', fontsize=14)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('RMSE [m]')
    ax2.legend()
    ax2.grid(True)

    # Statistics for the report
    final_rmse_ekf = rmse_ekf_running.iloc[-1]
    final_rmse_kf = rmse_kf_running.iloc[-1]
    improvement = ((final_rmse_ekf - final_rmse_kf) / final_rmse_ekf) * 100

    # Add text box with final stats
    stats_text = (f"FINAL PERFORMANCE:\n"
                  f"Raw EKF RMSE: {final_rmse_ekf:.4f} m\n"
                  f"Fused KF RMSE: {final_rmse_kf:.4f} m\n"
                  f"Precision Gain: {improvement:.2f}%")
    
    plt.gcf().text(0.02, 0.02, stats_text, fontsize=12, bbox=dict(facecolor='white', alpha=0.8, edgecolor='black'))

    plt.tight_layout(rect=[0, 0.05, 1, 0.95])
    plt.savefig('benchmarking_comparison24.png')
    print(f"Plot saved. Final KF RMSE: {final_rmse_kf:.4f}m")
    plt.show()

if __name__ == "__main__":
    plot_benchmarking_results('global_tracking_comparison24.csv')

"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_benchmarking_results(csv_file):
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: {csv_file} not found.")
        return

    # Normalize time
    df['timestamp'] = df['timestamp'] - df['timestamp'].iloc[0]

    # Calculate Euclidean Errors
    err_ekf = np.sqrt((df['ekf_raw_x'] - df['gt_x'])**2 + (df['ekf_raw_y'] - df['gt_y'])**2)
    err_kf = np.sqrt((df['kf_x'] - df['gt_x'])**2 + (df['kf_y'] - df['gt_y'])**2)

    # Calculate RMSE
    rmse_ekf_running = np.sqrt(np.cumsum(err_ekf**2) / (np.arange(len(err_ekf)) + 1))
    rmse_kf_running = np.sqrt(np.cumsum(err_kf**2) / (np.arange(len(err_kf)) + 1))

    # --- WINDOW 1: Standard Comparative Trajectories (Auto-Scaled) ---
    plt.figure(num='Window 1: Auto-Scaled Trajectories', figsize=(12, 8))
    plt.plot(df['gt_tb1_x'], df['gt_tb1_y'], color='black', linestyle='--', label='Observer (TB1) GT', alpha=0.3)
    plt.plot(df['gt_x'], df['gt_y'], color='green', label='Target (TB3) GT', linewidth=3)
    plt.plot(df['ekf_raw_x'], df['ekf_raw_y'], color='blue', linestyle='--', label='TB3 Raw EKF (Odom)', alpha=0.7)
    plt.plot(df['kf_x'], df['kf_y'], color='red', linestyle=':', label='TB3 Fused KF (Lidar)', linewidth=2)
    
    # Direction Markers
    plt.plot(df['gt_x'].iloc[0], df['gt_y'].iloc[0], 'go', label='Start')
    plt.plot(df['gt_x'].iloc[-1], df['gt_y'].iloc[-1], 'ro', label='End')
    
    plt.title('Comparative Path Analysis (Zoomed View)')
    plt.xlabel('Global X [m]')
    plt.ylabel('Global Y [m]')
    plt.legend()
    plt.grid(True)

    # --- WINDOW 2: Fixed Lab Frame (The 4x4m Arena) ---
    plt.figure(num='Window 2: Fixed Lab Frame (4x4m)', figsize=(10, 10))
    ax_fix = plt.gca()
    ax_fix.set_xlim([-2.0, 2.0])
    ax_fix.set_ylim([-2.0, 2.0])
    ax_fix.set_aspect('equal', adjustable='box')
    
    # Observer
    plt.plot(df['gt_tb1_x'], df['gt_tb1_y'], color='gray', linestyle='--', label='Observer GT', alpha=0.3)
    # Target Paths
    plt.plot(df['gt_x'], df['gt_y'], color='green', label='Target GT', linewidth=3)
    plt.plot(df['ekf_raw_x'], df['ekf_raw_y'], color='blue', linestyle='--', label='Target Raw EKF (Odom)', alpha=0.5)
    plt.plot(df['kf_x'], df['kf_y'], color='red', linestyle=':', label='Target Fused KF', linewidth=2)
    
    plt.title('Fixed Lab Reference Frame (-2m to 2m)')
    plt.xlabel('Global X [m]')
    plt.ylabel('Global Y [m]')
    plt.legend(loc='upper right')
    plt.grid(True, which='both', linestyle='--', alpha=0.5)

    # --- WINDOW 3: RMSE Analysis ---
    plt.figure(num='Window 3: Accuracy Metrics', figsize=(12, 6))
    plt.plot(df['timestamp'], rmse_ekf_running, color='blue', label='Running RMSE: Raw EKF', alpha=0.6)
    plt.plot(df['timestamp'], rmse_kf_running, color='red', label='Running RMSE: Fused KF', linewidth=2)
    
    plt.title('System Accuracy Benchmarking (RMSE)')
    plt.xlabel('Time [s]')
    plt.ylabel('RMSE [m]')
    plt.legend()
    plt.grid(True)

    # Stats Calculation
    final_rmse_ekf = rmse_ekf_running.iloc[-1]
    final_rmse_kf = rmse_kf_running.iloc[-1]
    improvement = ((final_rmse_ekf - final_rmse_kf) / final_rmse_ekf) * 100

    stats_text = (f"FINAL PERFORMANCE SUMMARY:\n"
                  f"Raw EKF Final RMSE: {final_rmse_ekf:.4f} m\n"
                  f"Fused KF Final RMSE: {final_rmse_kf:.4f} m\n"
                  f"Accuracy Gain: {improvement:.2f}%")
    
    plt.gcf().text(0.15, 0.05, stats_text, fontsize=11, 
                   bbox=dict(facecolor='white', alpha=0.9, edgecolor='black'))

    print(f"Analysis Complete. Displaying 3 Windows.")
    plt.show()

if __name__ == "__main__":
    plot_benchmarking_results('tracking_dbscan6.csv')