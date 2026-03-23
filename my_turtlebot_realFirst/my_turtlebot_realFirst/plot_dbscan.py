import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_benchmarking_results(csv_file):
    try:
        # Expected headers: ts, leg_x, leg_y, db_x, db_y, odom_x, odom_y, gt_x, gt_y, gt_obs_x, gt_obs_y
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: {csv_file} not found.")
        return

    # Normalize time
    df['ts'] = df['ts'] - df['ts'].iloc[0]

    # --- 1. Calculate Euclidean Errors ---
    err_odom = np.sqrt((df['odom_x'] - df['gt_x'])**2 + (df['odom_y'] - df['gt_y'])**2)
    err_leg  = np.sqrt((df['leg_x'] - df['gt_x'])**2 + (df['leg_y'] - df['gt_y'])**2)
    err_db   = np.sqrt((df['db_x'] - df['gt_x'])**2 + (df['db_y'] - df['gt_y'])**2)

    # --- 2. Calculate Running RMSE ---
    def get_rmse(err):
        return np.sqrt(np.cumsum(err**2) / (np.arange(len(err)) + 1))

    rmse_odom = get_rmse(err_odom)
    rmse_leg  = get_rmse(err_leg)
    rmse_db   = get_rmse(err_db)

    # --- WINDOW 1: Comparative Trajectories (Zoomed) ---
    plt.figure(num='Window 1: Trajectory Comparison', figsize=(12, 8))
    plt.plot(df['gt_obs_x'], df['gt_obs_y'], color='black', linestyle='--', label='Observer (TB1) GT', alpha=0.3)
    plt.plot(df['gt_x'], df['gt_y'], color='green', label='Target (TB3) GT', linewidth=3)
    plt.plot(df['odom_x'], df['odom_y'], color='blue', linestyle='--', label='Raw Odom (EKF)', alpha=0.5)
    
    # The two competitors
    plt.plot(df['leg_x'], df['leg_y'], color='red', linestyle=':', label='Legacy Gating KF', linewidth=2)
    plt.plot(df['db_x'], df['db_y'], color='purple', linestyle='-', label='DBSCAN Cluster KF', linewidth=1.5, alpha=0.8)
    
    plt.plot(df['gt_x'].iloc[0], df['gt_y'].iloc[0], 'go', label='Start')
    plt.plot(df['gt_x'].iloc[-1], df['gt_y'].iloc[-1], 'ro', label='End')
    
    plt.title('Path Comparison: Legacy vs. DBSCAN', fontsize=14)
    plt.xlabel('Global X [m]'); plt.ylabel('Global Y [m]')
    plt.legend(); plt.grid(True)

    # --- WINDOW 2: Fixed Lab Frame ---
    plt.figure(num='Window 2: Fixed Lab Frame', figsize=(10, 10))
    ax = plt.gca()
    ax.set_xlim([-2.0, 2.0]); ax.set_ylim([-2.0, 2.0])
    ax.set_aspect('equal')
    
    plt.plot(df['gt_x'], df['gt_y'], color='green', label='Target GT', linewidth=3)
    plt.plot(df['leg_x'], df['leg_y'], color='red', linestyle=':', label='Legacy KF')
    plt.plot(df['db_x'], df['db_y'], color='purple', label='DBSCAN KF')
    
    plt.title('Characterization in 4x4m Arena')
    plt.legend(); plt.grid(True, linestyle='--', alpha=0.5)

    # --- WINDOW 3: RMSE Comparison ---
    plt.figure(num='Window 3: Accuracy Metrics', figsize=(12, 7))
    plt.plot(df['ts'], rmse_odom, color='blue', label='RMSE: Raw Odom', alpha=0.4)
    plt.plot(df['ts'], rmse_leg, color='red', label='RMSE: Legacy Method', linewidth=2)
    plt.plot(df['ts'], rmse_db, color='purple', label='RMSE: DBSCAN Method', linewidth=2)
    
    plt.title('Algorithm Accuracy Characterization (RMSE)', fontsize=14)
    plt.xlabel('Time [s]'); plt.ylabel('RMSE [m]')
    plt.legend(); plt.grid(True)

    # Performance Summary Text
    f_odom = rmse_odom.iloc[-1]
    f_leg  = rmse_leg.iloc[-1]
    f_db   = rmse_db.iloc[-1]
    
    gain_leg = ((f_odom - f_leg) / f_odom) * 100
    gain_db  = ((f_odom - f_db) / f_odom) * 100

    stats_text = (f"FINAL CHARACTERIZATION:\n"
                  f"Raw Odom RMSE: {f_odom:.4f} m\n"
                  f"Legacy KF RMSE: {f_leg:.4f} m (Gain: {gain_leg:.1f}%)\n"
                  f"DBSCAN KF RMSE: {f_db:.4f} m (Gain: {gain_db:.1f}%)")
    
    plt.gcf().text(0.15, 0.05, stats_text, fontsize=11, 
                   bbox=dict(facecolor='white', alpha=0.9, edgecolor='black'))

    print("Characterization Complete. Compare the Red (Legacy) and Purple (DBSCAN) lines.")
    plt.show()

if __name__ == "__main__":
    # Ensure this matches the filename in your KalmanObserver node
    plot_benchmarking_results('dbscan_vs_legacy_comparison7.csv')
