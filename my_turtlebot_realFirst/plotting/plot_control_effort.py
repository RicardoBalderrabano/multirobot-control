import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_unified_control_effort(filename='swarm_control_effort.csv'):
    if not os.path.exists(filename):
        print(f"Error: {filename} not found. Have you run the swarm yet?")
        return

    # Read the single unified CSV
    df = pd.read_csv(filename)
    if df.empty:
        print("CSV is empty.")
        return

    # Normalize time so the graph always starts at 0 seconds
    df['timestamp'] = df['timestamp'] - df['timestamp'].min()
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.canvas.manager.set_window_title('Swarm Control Effort')
    fig.suptitle('Commanded Velocities Over Time (Unified File)', fontsize=16, fontweight='bold')
    
    colors = {'tb1': 'red', 'tb2': 'blue', 'tb3': 'green'}
    
    # Group the single dataframe by the robot_id column
    for rid, group in df.groupby('robot_id'):
        color = colors.get(rid, 'black') # Fallback to black if ID isn't 1, 2, or 3
        
        # Plot Linear Velocity (v)
        ax1.plot(group['timestamp'], group['v_cmd'], label=f'{rid.upper()} v_cmd', color=color, linewidth=1.5)
        
        # Plot Angular Velocity (w)
        ax2.plot(group['timestamp'], group['w_cmd'], label=f'{rid.upper()} w_cmd', color=color, linewidth=1.5)

    # Format Linear Velocity Subplot
    ax1.set_title('Linear Velocity (v)')
    ax1.set_ylabel('Velocity (m/s)')
    ax1.axhline(0.22, color='black', linestyle='--', alpha=0.5, label='Max Velocity (+0.22)')
    ax1.axhline(-0.22, color='black', linestyle='--', alpha=0.5, label='Max Velocity (-0.22)')
    ax1.grid(True, linestyle=':', alpha=0.7)
    ax1.legend(loc='upper right')

    # Format Angular Velocity Subplot
    ax2.set_title('Angular Velocity (ω)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (rad/s)')
    ax2.axhline(1.5, color='black', linestyle='--', alpha=0.5, label='Max Angular (+1.5)')
    ax2.axhline(-1.5, color='black', linestyle='--', alpha=0.5, label='Max Angular (-1.5)')
    ax2.grid(True, linestyle=':', alpha=0.7)
    ax2.legend(loc='upper right')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_unified_control_effort()
