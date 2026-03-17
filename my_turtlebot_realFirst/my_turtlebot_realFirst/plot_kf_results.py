
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
    plt.savefig('benchmarking_comparison13.png')
    print(f"Plot saved. Final KF RMSE: {final_rmse_kf:.4f}m")
    plt.show()

if __name__ == "__main__":
    plot_benchmarking_results('global_tracking_comparison12.csv')