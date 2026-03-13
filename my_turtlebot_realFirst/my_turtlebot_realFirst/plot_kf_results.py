"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_global_results(csv_file):
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: {csv_file} not found. Ensure the Kalman node is running and saving data.")
        return

    # Normalize time
    df['timestamp'] = df['timestamp'] - df['timestamp'].iloc[0]

    # Create Figure
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle('Multi-Robot Global State Estimation Performance', fontsize=16)

    # --- Plot 1: Global Trajectories (The World Map) ---
    ax1 = fig.add_subplot(221)
    # Observer Path
    ax1.plot(df['gt_tb1_x'], df['gt_tb1_y'], color='gray', linestyle='--', label='Observer (TB1) Ground Truth', alpha=0.6)
    # Target Path (Ground Truth vs KF)
    ax1.plot(df['gt_tb3_x'], df['gt_tb3_y'], color='green', label='Target (TB3) Ground Truth', linewidth=2)
    ax1.plot(df['kf_global_x'], df['kf_global_y'], color='red', linestyle=':', label='KF Global Estimate', linewidth=2)
    
    ax1.set_title('Global World Trajectories')
    ax1.set_xlabel('Global X [m]')
    ax1.set_ylabel('Global Y [m]')
    ax1.legend()
    ax1.grid(True)

    # --- Plot 2: X and Y Error Components ---
    ax2 = fig.add_subplot(222)
    err_x = df['kf_global_x'] - df['gt_tb3_x']
    err_y = df['kf_global_y'] - df['gt_tb3_y']
    
    ax2.plot(df['timestamp'], err_x, label='Error X', color='blue', alpha=0.7)
    ax2.plot(df['timestamp'], err_y, label='Error Y', color='orange', alpha=0.7)
    ax2.set_title('Coordinate Error Components')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Error [m]')
    ax2.legend()
    ax2.grid(True)

    # --- Plot 3: Euclidean (Total) Error ---
    ax3 = fig.add_subplot(212)
    l2_error = np.sqrt(err_x**2 + err_y**2)
    mean_rmse = np.sqrt(np.mean(l2_error**2))
    
    ax3.plot(df['timestamp'], l2_error, color='purple', label='Total Euclidean Error')
    ax3.axhline(y=mean_rmse, color='red', linestyle='--', label=f'RMSE: {mean_rmse:.4f}m')
    
    ax3.set_title('Total Estimation Error (RMSE Analysis)')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Distance Error [m]')
    ax3.legend()
    ax3.grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig('global_kf_analysis2.png')
    print(f"Analysis complete. RMSE: {mean_rmse:.4f}m. Plot saved as 'global_kf_analysis.png'")
    plt.show()

if __name__ == "__main__":
    plot_global_results('global_tracking_results2.csv')
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_comparison_results(csv_file):
    try:
        # Load the new CSV with ekf_raw columns
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: {csv_file} not found. Check the filename in your Kalman node.")
        return

    # Normalize time
    df['timestamp'] = df['timestamp'] - df['timestamp'].iloc[0]

    # Create Figure
    fig = plt.figure(figsize=(15, 12))
    fig.suptitle('Performance Comparison: Raw EKF vs. Fused Kalman Filter', fontsize=18)

    # --- Plot 1: Trajectory Comparison (Top Down) ---
    ax1 = fig.add_subplot(221)
    # Ground Truth (The Goal)
    ax1.plot(df['gt_x'], df['gt_y'], color='green', label='OptiTrack (Ground Truth)', linewidth=3, alpha=0.7)
    # Raw EKF (Prediction only - will likely show drift)
    ax1.plot(df['ekf_raw_x'], df['ekf_raw_y'], color='blue', linestyle='--', label='Raw EKF (Odom Only)', linewidth=1.5)
    # Fused KF (The Result)
    ax1.plot(df['kf_x'], df['kf_y'], color='red', linestyle=':', label='Fused KF (Odom + LiDAR)', linewidth=2)
    
    ax1.set_title('Global Path Comparison')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.legend()
    ax1.grid(True)

    # --- Plot 2: Cumulative Displacement Error ---
    ax2 = fig.add_subplot(222)
    # Calculate Euclidean errors
    err_ekf = np.sqrt((df['ekf_raw_x'] - df['gt_x'])**2 + (df['ekf_raw_y'] - df['gt_y'])**2)
    err_kf = np.sqrt((df['kf_x'] - df['gt_x'])**2 + (df['kf_y'] - df['gt_y'])**2)
    
    ax2.plot(df['timestamp'], err_ekf, color='blue', label='Raw EKF Error', alpha=0.6)
    ax2.plot(df['timestamp'], err_kf, color='red', label='Fused KF Error', linewidth=2)
    ax2.set_title('Error Comparison Over Time')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Euclidean Error [m]')
    ax2.legend()
    ax2.grid(True)

    # --- Plot 3: Error Components (FUSION IMPACT) ---
    ax3 = fig.add_subplot(212)
    # Show the reduction in error
    error_reduction = err_ekf - err_kf
    ax3.fill_between(df['timestamp'], 0, error_reduction, color='cyan', alpha=0.3, label='LiDAR Correction Benefit')
    ax3.plot(df['timestamp'], error_reduction, color='darkcyan', linewidth=1)
    
    ax3.set_title('Correction Magnitude (How much LiDAR improved the estimate)')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Error Reduction [m]')
    ax3.legend()
    ax3.grid(True)

    # Calculate Statistics for Report
    rmse_ekf = np.sqrt(np.mean(err_ekf**2))
    rmse_kf = np.sqrt(np.mean(err_kf**2))
    improvement = ((rmse_ekf - rmse_kf) / rmse_ekf) * 100

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Text summary on plot
    summary_text = f"RMSE EKF: {rmse_ekf:.4f}m\nRMSE KF: {rmse_kf:.4f}m\nTotal Improvement: {improvement:.2f}%"
    fig.text(0.02, 0.02, summary_text, fontsize=12, bbox=dict(facecolor='white', alpha=0.8))

    plt.savefig('ekf_vs_kf_comparison.png')
    print(f"Analysis complete.\n- EKF RMSE: {rmse_ekf:.4f}m\n- KF RMSE: {rmse_kf:.4f}m\n- Improvement: {improvement:.2f}%")
    plt.show()

if __name__ == "__main__":
    # Ensure this matches the csv_filename in your ROS node
    plot_comparison_results('global_tracking_comparison.csv')