#!/usr/bin/env python3

import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm
import numpy as np
import re
import time

def plot_sun_vectors(port='/dev/tty.usbmodem101', baudrate=115200):
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(12, 10), facecolor='black')
    ax = fig.add_subplot(111, projection='3d', facecolor='black')
    
    # Create a more detailed sphere with better resolution
    u = np.linspace(0, 2 * np.pi, 50)
    v = np.linspace(0, np.pi, 50)
    x_sphere = np.outer(np.cos(u), np.sin(v))
    y_sphere = np.outer(np.sin(u), np.sin(v))
    z_sphere = np.outer(np.ones(np.size(u)), np.cos(v))
    
    # Enhanced sphere visualization
    ax.plot_wireframe(x_sphere, y_sphere, z_sphere, alpha=0.15, color='cyan', linewidth=0.5)
    ax.plot_surface(x_sphere, y_sphere, z_sphere, alpha=0.05, color='white', shade=True)
    
    # Enhanced reference frame axes with labels
    ax.quiver(0, 0, 0, 1.2, 0, 0, color='#FF6B6B', linewidth=3, arrow_length_ratio=0.1)
    ax.quiver(0, 0, 0, 0, 1.2, 0, color='#4ECDC4', linewidth=3, arrow_length_ratio=0.1)
    ax.quiver(0, 0, 0, 0, 0, 1.2, color='#45B7D1', linewidth=3, arrow_length_ratio=0.1)
    
    # Add axis labels
    ax.text(1.3, 0, 0, 'X', color='#FF6B6B', fontsize=12, weight='bold')
    ax.text(0, 1.3, 0, 'Y', color='#4ECDC4', fontsize=12, weight='bold')
    ax.text(0, 0, 1.3, 'Z', color='#45B7D1', fontsize=12, weight='bold')
    
    # Enhanced styling
    ax.set_xlim(-1.3, 1.3)
    ax.set_ylim(-1.3, 1.3)
    ax.set_zlim(-1.3, 1.3)
    ax.set_xlabel('X', color='white', fontsize=10)
    ax.set_ylabel('Y', color='white', fontsize=10)
    ax.set_zlabel('Z', color='white', fontsize=10)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('gray')
    ax.yaxis.pane.set_edgecolor('gray')
    ax.zaxis.pane.set_edgecolor('gray')
    ax.xaxis.pane.set_alpha(0.1)
    ax.yaxis.pane.set_alpha(0.1)
    ax.zaxis.pane.set_alpha(0.1)
    ax.grid(True, alpha=0.3)
    
    # Add title
    plt.title('Sun Vector Visualization', color='white', fontsize=16, weight='bold', pad=20)
    
    plt.ion()
    plt.show(block=False)
    
    pattern = r'\[DEBUG\].*Sun vector: \[([-\d.]+), ([-\d.]+), ([-\d.]+)\]'
    arrow_history = []  # Store (arrow, glow_arrow, timestamp) tuples
    last_time = 0
    fade_duration = 0.2  # 200ms

    with serial.Serial(port, baudrate, timeout=0.01) as ser:
        while True:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                match = re.search(pattern, line)
                
                # Clean up old arrows and update colors
                current_time = time.time()
                arrows_to_remove = []
                
                for i, (main_arrow, glow_arrow, timestamp) in enumerate(arrow_history):
                    age = current_time - timestamp
                    if age > fade_duration:
                        # Remove expired arrows
                        try:
                            main_arrow.remove()
                            glow_arrow.remove()
                        except:
                            pass
                        arrows_to_remove.append(i)
                    else:
                        # Update color based on age using viridis
                        fade_ratio = age / fade_duration  # 0 (new) to 1 (old)
                        color_value = 1.0 - fade_ratio  # 1 (new) to 0 (old)
                        color = cm.viridis(color_value)
                        
                        # Update arrow colors
                        try:
                            main_arrow.set_color(color)
                            main_arrow.set_alpha(0.9 * color_value)
                            glow_arrow.set_color(color)
                            glow_arrow.set_alpha(0.3 * color_value)
                        except:
                            pass
                
                # Remove expired arrows from history
                for i in reversed(arrows_to_remove):
                    arrow_history.pop(i)
                
                if match and (current_time - last_time) > 0.01:
                    x, y, z = map(float, match.groups())
                    print(f"Sun vector: [{x:.6f}, {y:.6f}, {z:.6f}]")
                    
                    # Calculate magnitude for color intensity
                    magnitude = np.sqrt(x**2 + y**2 + z**2)
                    
                    # PLOT new vector (including zero vectors)
                    if magnitude > 1e-6:  # Non-zero vector
                        # Use viridis color at full intensity for new arrows
                        arrow_color = cm.viridis(1.0)  # Brightest viridis color
                        
                        # Enhanced arrow with better visibility
                        main_arrow = ax.quiver(0, 0, 0, x, y, z, 
                                             color=arrow_color, 
                                             linewidth=5,
                                             arrow_length_ratio=0.15,
                                             alpha=0.9,
                                             pivot='tail')
                        
                        # Add a subtle glow effect
                        glow_arrow = ax.quiver(0, 0, 0, x, y, z,
                                             color=arrow_color,
                                             linewidth=8,
                                             arrow_length_ratio=0.15,
                                             alpha=0.3,
                                             pivot='tail')
                        
                        # Add to history with timestamp
                        arrow_history.append((main_arrow, glow_arrow, current_time))
                        
                        print(f"Plotted sun vector arrow (magnitude: {magnitude:.3f})")
                    else:
                        print("Zero vector - no arrow")
                    
                    plt.draw()
                    plt.pause(0.005)  # Slightly longer pause for smoother animation
                    last_time = current_time
                    
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    plot_sun_vectors(port='/dev/tty.usbmodem101')