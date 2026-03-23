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

    # Euclidean Errors
    err_ekf = np.sqrt((df['ekf_raw_x'] - df['gt_x'])**2 + (df['ekf_raw_y'] - df['gt_y'])**2)
    err_kf = np.sqrt((df['kf_x'] - df['gt_x'])**2 + (df['kf_y'] - df['gt_y'])**2)

    # Running RMSE
    rmse_ekf_running = np.sqrt(np.cumsum(err_ekf**2) / (np.arange(len(err_ekf)) + 1))
    rmse_kf_running = np.sqrt(np.cumsum(err_kf**2) / (np.arange(len(err_kf)) + 1))

    # --- WINDOW 1 ---
    plt.figure(num='Window 1: Trajectories', figsize=(12, 8))
    # Use 'gt_obs_x' to match your CSV
    plt.plot(df['gt_obs_x'], df['gt_obs_y'], color='black', linestyle='--', label='Observer (TB1) GT', alpha=0.3)
    plt.plot(df['gt_x'], df['gt_y'], color='green', label='Target (TB3) GT', linewidth=3)
    plt.plot(df['ekf_raw_x'], df['ekf_raw_y'], color='blue', linestyle='--', label='TB3 Raw EKF', alpha=0.7)
    plt.plot(df['kf_x'], df['kf_y'], color='red', linestyle=':', label='TB3 Fused KF', linewidth=2)
    plt.legend()
    plt.grid(True)

    # --- WINDOW 3: RMSE ---
    plt.figure(num='Window 3: Accuracy Metrics', figsize=(12, 6))
    plt.plot(df['timestamp'], rmse_ekf_running, color='blue', label='RMSE: Raw EKF')
    plt.plot(df['timestamp'], rmse_kf_running, color='red', label='RMSE: Fused KF', linewidth=2)
    
    final_rmse_ekf = rmse_ekf_running.iloc[-1]
    final_rmse_kf = rmse_kf_running.iloc[-1]
    improvement = ((final_rmse_ekf - final_rmse_kf) / final_rmse_ekf) * 100

    stats_text = (f"PERFORMANCE SUMMARY:\n"
                  f"Raw EKF Final RMSE: {final_rmse_ekf:.4f} m\n"
                  f"Fused KF Final RMSE: {final_rmse_kf:.4f} m\n"
                  f"Accuracy Gain: {improvement:.2f}%")
    
    plt.gcf().text(0.15, 0.05, stats_text, bbox=dict(facecolor='white', alpha=0.9))
    plt.title('Accuracy Benchmarking (RMSE)')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    plot_benchmarking_results('tracking_dbscan22.csv')