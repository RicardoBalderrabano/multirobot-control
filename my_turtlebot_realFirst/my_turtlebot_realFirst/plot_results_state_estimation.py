"""

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

def plot_trajectories(csv_file):
    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} not found! Check your path.")
        return

    # Load the data
    data = pd.read_csv(csv_file)
    
    # --- PHYSICAL COORDINATE ALIGNMENT ---
    # We want TB1 (the observer) to be at (0,0).
    # Based on your setup, TB2 starts at this offset relative to TB1:
    tb2_initial_x = 0.39
    tb2_initial_y = 0.006
    
    # 1. Align OptiTrack Ground Truth
    # Shift OptiTrack so its first point matches the TB2 initial offset
    opti_start_x = data['opti_x'].iloc[0]
    opti_start_y = data['opti_y'].iloc[0]
    
    data['opti_x_aligned'] = (data['opti_x'] - opti_start_x) + tb2_initial_x
    data['opti_y_aligned'] = (data['opti_y'] - opti_start_y) + tb2_initial_y
    
    # 2. Kalman and Odom are already calculated relative to TB1 in your node
    # We do NOT normalize these to (0,0) because they already start at the offset.
    # We only clean up potential startup noise by ensuring they start exactly at the defined pos.
    data['kf_x_clean'] = data['kf_x']
    data['kf_y_clean'] = data['kf_y']
    data['odom_x_clean'] = data['odom_x']
    data['odom_y_clean'] = data['odom_y']
    data['lidar_x_clean'] = data['lidar_x']
    data['lidar_y_clean'] = data['lidar_y']

    plt.figure(figsize=(12, 8))
    
    # --- PLOTTING ---
    # 0. Plot TB1 at the Origin for reference
    plt.plot(0, 0, 'ms', markersize=10, label='TB1 Observer (Origin)')
    
    # 1. Ground Truth (OptiTrack) - Solid Black
    plt.plot(data['opti_x_aligned'], data['opti_y_aligned'], 'k-', linewidth=3, label='Ground Truth (OptiTrack)')
    
    # 2. Odometry Only (Red dashed line)
    plt.plot(data['odom_x_clean'], data['odom_y_clean'], 'r--', label='Odometry (Drifted)', alpha=0.7)
    
    # 3. Raw LIDAR Points (Blue dots)
    plt.scatter(data['lidar_x_clean'], data['lidar_y_clean'], c='blue', s=8, label='Raw LIDAR (Noisy)', alpha=0.4)
    
    # 4. Kalman Filter Result (Green solid line)
    plt.plot(data['kf_x_clean'], data['kf_y_clean'], 'g-', linewidth=2.5, label='Kalman Filter (Fused)')

    plt.title('Multi-Robot Tracking: Robot 2 Position Relative to Robot 1', fontsize=14)
    plt.xlabel('X Position relative to TB1 (meters)', fontsize=12)
    plt.ylabel('Y Position relative to TB1 (meters)', fontsize=12)
    plt.legend(loc='best')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.axis('equal') 
    
    # --- Error Calculation vs Aligned Truth ---
    kf_error = np.sqrt((data['kf_x_clean'] - data['opti_x_aligned'])**2 + 
                       (data['kf_y_clean'] - data['opti_y_aligned'])**2)
    rmse_kf = np.sqrt(np.mean(kf_error**2))
    
    odom_error = np.sqrt((data['odom_x_clean'] - data['opti_x_aligned'])**2 + 
                         (data['odom_y_clean'] - data['opti_y_aligned'])**2)
    rmse_odom = np.sqrt(np.mean(odom_error**2))

    print(f"--- Performance Metrics ---")
    print(f"Kalman Filter RMSE: {rmse_kf:.4f} meters")
    print(f"Raw Odometry RMSE:  {rmse_odom:.4f} meters")
    print(f"Improvement:        {((rmse_odom - rmse_kf) / rmse_odom * 100):.1f}%")

    plt.savefig('final_tracking_comparison_aligned35.png', dpi=300)
    plt.show()

if __name__ == "__main__":
    plot_trajectories('tracking_results_with_optitrack35.csv')

"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches # Added for robot footprint
import pandas as pd
import numpy as np
import os

def plot_trajectories(csv_file):
    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} not found! Check your path.")
        return

    data = pd.read_csv(csv_file)
    
    # --- PHYSICAL COORDINATE ALIGNMENT ---
    tb2_initial_x = 0.39
    tb2_initial_y = 0.006
    
    opti_start_x = data['opti_x'].iloc[0]
    opti_start_y = data['opti_y'].iloc[0]
    
    data['opti_x_aligned'] = (data['opti_x'] - opti_start_x) + tb2_initial_x
    data['opti_y_aligned'] = (data['opti_y'] - opti_start_y) + tb2_initial_y
    
    # Clean up columns
    data['kf_x_clean'] = data['kf_x']
    data['kf_y_clean'] = data['kf_y']
    data['odom_x_clean'] = data['odom_x']
    data['odom_y_clean'] = data['odom_y']
    data['lidar_x_clean'] = data['lidar_x']
    data['lidar_y_clean'] = data['lidar_y']

    fig, ax = plt.subplots(figsize=(12, 8)) # Use subplots to access 'ax'
    
    # --- PLOTTING ROBOT FOOTPRINT ---
    # Standard TurtleBot3 Burger radius is ~0.092m
    robot_radius = 0.105 
    
    # Draw TB1's physical body at the origin
    tb1_body = patches.Circle((0, 0), robot_radius, color='magenta', alpha=0.2, label='TB1 Physical Size')
    ax.add_patch(tb1_body)
    
    # Plot the center point (Observer)
    ax.plot(0, 0, 'ms', markersize=10, label='TB1 Observer (Origin)')
    
    # --- PLOTTING TRAJECTORIES ---
    ax.plot(data['opti_x_aligned'], data['opti_y_aligned'], 'k-', linewidth=3, label='Ground Truth (OptiTrack)')
    ax.plot(data['odom_x_clean'], data['odom_y_clean'], 'r--', label='Odometry (Drifted)', alpha=0.7)
    ax.scatter(data['lidar_x_clean'], data['lidar_y_clean'], c='blue', s=8, label='Raw LIDAR (Noisy)', alpha=0.4)
    ax.plot(data['kf_x_clean'], data['kf_y_clean'], 'g-', linewidth=2.5, label='Kalman Filter (Fused)')

    ax.set_title('Multi-Robot Tracking: Robot 2 Position Relative to Robot 1', fontsize=14)
    ax.set_xlabel('X Position relative to TB1 (meters)', fontsize=12)
    ax.set_ylabel('Y Position relative to TB1 (meters)', fontsize=12)
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.axis('equal') 
    
    # --- Error Calculation ---
    kf_error = np.sqrt((data['kf_x_clean'] - data['opti_x_aligned'])**2 + 
                       (data['kf_y_clean'] - data['opti_y_aligned'])**2)
    rmse_kf = np.sqrt(np.mean(kf_error**2))
    
    odom_error = np.sqrt((data['odom_x_clean'] - data['opti_x_aligned'])**2 + 
                         (data['odom_y_clean'] - data['opti_y_aligned'])**2)
    rmse_odom = np.sqrt(np.mean(odom_error**2))

    print(f"--- Performance Metrics ---")
    print(f"Kalman Filter RMSE: {rmse_kf:.4f} meters")
    print(f"Raw Odometry RMSE:  {rmse_odom:.4f} meters")
    print(f"Improvement:        {((rmse_odom - rmse_kf) / rmse_odom * 100):.1f}%")

    plt.savefig('final_tracking_with_footprint36.png', dpi=300)
    plt.show()

if __name__ == "__main__":
    plot_trajectories('tracking_results_with_optitrack36.csv')