#!/usr/bin/env python3

import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm
import numpy as np
import re
import time
from collections import deque

def plot_sun_vectors(port='/dev/tty.usbmodem101', baudrate=115200):
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(12, 10), facecolor='black')
    ax = fig.add_subplot(111, projection='3d', facecolor='black')
    
    # Simplified sphere visualization for better performance
    u = np.linspace(0, 2 * np.pi, 20)  # Reduced resolution
    v = np.linspace(0, np.pi, 20)
    x_sphere = np.outer(np.cos(u), np.sin(v))
    y_sphere = np.outer(np.sin(u), np.sin(v))
    z_sphere = np.outer(np.ones(np.size(u)), np.cos(v))
    
    # Only wireframe for better performance
    ax.plot_wireframe(x_sphere, y_sphere, z_sphere, alpha=0.1, color='cyan', linewidth=0.3)
    
    # Reference frame axes
    ax.quiver(0, 0, 0, 1.2, 0, 0, color='#FF6B6B', linewidth=2, arrow_length_ratio=0.1)
    ax.quiver(0, 0, 0, 0, 1.2, 0, color='#4ECDC4', linewidth=2, arrow_length_ratio=0.1)
    ax.quiver(0, 0, 0, 0, 0, 1.2, color='#45B7D1', linewidth=2, arrow_length_ratio=0.1)
    
    # Axis labels
    ax.text(1.3, 0, 0, 'X', color='#FF6B6B', fontsize=10, weight='bold')
    ax.text(0, 1.3, 0, 'Y', color='#4ECDC4', fontsize=10, weight='bold')
    ax.text(0, 0, 1.3, 'Z', color='#45B7D1', fontsize=10, weight='bold')
    
    # Styling
    ax.set_xlim(-1.3, 1.3)
    ax.set_ylim(-1.3, 1.3)
    ax.set_zlim(-1.3, 1.3)
    ax.set_xlabel('X', color='white', fontsize=8)
    ax.set_ylabel('Y', color='white', fontsize=8)
    ax.set_zlabel('Z', color='white', fontsize=8)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.grid(True, alpha=0.2)
    
    plt.title('Sun Vector Visualization', color='white', fontsize=14, weight='bold')
    
    plt.ion()
    plt.show(block=False)
    
    pattern = r'\[DEBUG\].*Sun vector: \[([-\d.]+), ([-\d.]+), ([-\d.]+)\]'
    arrow_history = deque()  # Use deque for efficient operations
    last_plot_time = 0
    last_cleanup_time = 0
    cleanup_interval = 0.15  # 150ms cleanup interval
    plot_interval = 1.0 / 60.0  # 60Hz = ~16.67ms
    
    # Pre-compute colors for efficiency
    fade_steps = 9  # Number of fade levels (150ms / 16.67ms ≈ 9)
    fade_colors = [cm.viridis(1.0 - i / fade_steps) for i in range(fade_steps)]
    fade_alphas_main = [0.9 * (1.0 - i / fade_steps) for i in range(fade_steps)]
    fade_alphas_glow = [0.3 * (1.0 - i / fade_steps) for i in range(fade_steps)]

    with serial.Serial(port, baudrate, timeout=0.001) as ser:  # Shorter timeout
        while True:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                match = re.search(pattern, line)
                current_time = time.time()
                
                # Efficient cleanup every 150ms
                if current_time - last_cleanup_time > cleanup_interval:
                    arrows_to_remove = []
                    for i, (main_arrow, glow_arrow, timestamp, _) in enumerate(arrow_history):
                        if current_time - timestamp > cleanup_interval:
                            try:
                                main_arrow.remove()
                                glow_arrow.remove()
                            except:
                                pass
                            arrows_to_remove.append(i)
                    
                    # Remove expired arrows
                    for i in reversed(arrows_to_remove):
                        del arrow_history[i]
                    
                    last_cleanup_time = current_time
                
                # Plot at 60Hz
                if match and (current_time - last_plot_time) >= plot_interval:
                    x, y, z = map(float, match.groups())
                    magnitude = np.sqrt(x**2 + y**2 + z**2)
                    
                    if magnitude > 1e-6:  # Non-zero vector
                        # Use brightest color for new arrows
                        arrow_color = fade_colors[0]
                        
                        # Create new arrows with minimal styling for performance
                        main_arrow = ax.quiver(0, 0, 0, x, y, z, 
                                             color=arrow_color, 
                                             linewidth=3,
                                             arrow_length_ratio=0.12,
                                             alpha=fade_alphas_main[0])
                        
                        glow_arrow = ax.quiver(0, 0, 0, x, y, z,
                                             color=arrow_color,
                                             linewidth=5,
                                             arrow_length_ratio=0.12,
                                             alpha=fade_alphas_glow[0])
                        
                        # Add to history
                        arrow_history.append((main_arrow, glow_arrow, current_time, 0))
                    
                    # Minimal redraw
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()
                    last_plot_time = current_time
                    
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    plot_sun_vectors(port='/dev/tty.usbmodem101')