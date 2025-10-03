#python3 sensor_occlusion_sphere.py sensor_occlusion_sphere_monte_carlo.csv --colormap viridis --resolution 120 --interpolation nearest --show-histogram

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata
from scipy.spatial import SphericalVoronoi
import argparse
import sys
import os

def read_csv_data(filepath):
    """
    Read the CSV file containing Sun vectors and sensor illumination data.
    
    Args:
        filepath (str): Path to the CSV file
    
    Returns:
        pd.DataFrame: Loaded data
    """
    try:
        data = pd.read_csv(filepath)
        print(f"Successfully loaded CSV with {len(data)} rows and {len(data.columns)} columns")
        return data
    except FileNotFoundError:
        print(f"Error: File '{filepath}' not found.")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        sys.exit(1)

def define_sensor_pairs():
    """
    Define the sensor pairs based on the provided information.
    Returns a list of tuples, where each tuple contains the indices of paired sensors.
    """
    # Sensor pairs (assuming 0-based indexing, adjust if needed)
    # Y+ [9-10], Y- [11-12], Z+ [13-14], Z- [15-16]
    sensor_pairs = [
        (8, 9),   # Y+ pair
        (10, 11),  # Y- pair  
        (12, 13),  # Z+ pair
        (14, 15)   # Z- pair
    ]
    return sensor_pairs

def process_sun_vectors_and_sensors_unique(data):
    """
    Process the data to extract Sun vectors, normalize them, and count unique illuminated sensor directions.
    Accounts for sensor pairs that represent the same direction.
    
    Args:
        data (pd.DataFrame): Raw CSV data
    
    Returns:
        tuple: (normalized_vectors, unique_illuminated_counts, raw_illuminated_counts)
    """
    # Extract Sun vector columns
    sun_vectors = data[['Sun_X', 'Sun_Y', 'Sun_Z']].values
    
    # Normalize the Sun vectors to ensure they're unit vectors
    vector_magnitudes = np.linalg.norm(sun_vectors, axis=1)
    normalized_vectors = sun_vectors / vector_magnitudes.reshape(-1, 1)
    
    # Find sensor columns (all columns except Sun_X, Sun_Y, Sun_Z)
    sensor_columns = [col for col in data.columns if col.startswith('Sensor_')]
    
    if not sensor_columns:
        print("Warning: No sensor columns found. Looking for columns containing 'Sensor'...")
        sensor_columns = [col for col in data.columns if 'Sensor' in col or 'sensor' in col]
    
    if not sensor_columns:
        print("Error: No sensor columns found in the CSV file.")
        print("Available columns:", list(data.columns))
        sys.exit(1)
    
    print(f"Found {len(sensor_columns)} sensor columns")
    
    # Get sensor pairs
    sensor_pairs = define_sensor_pairs()
    
    # Verify sensor pairs are within the available sensor range
    max_sensor_idx = len(sensor_columns) - 1
    valid_pairs = []
    for pair in sensor_pairs:
        if pair[0] <= max_sensor_idx and pair[1] <= max_sensor_idx:
            valid_pairs.append(pair)
        else:
            print(f"Warning: Sensor pair {pair} exceeds available sensors (0-{max_sensor_idx}). Skipping.")
    
    sensor_pairs = valid_pairs
    print(f"Using {len(sensor_pairs)} valid sensor pairs for unique direction counting")
    
    # Get sensor data
    sensor_data = data[sensor_columns]
    
    # Count raw illuminated sensors (original method)
    raw_illuminated_counts = (sensor_data == 'ILLUMINATED').sum(axis=1).values
    
    # Count unique illuminated sensor directions
    unique_illuminated_counts = np.zeros(len(data))
    
    # Get all sensor indices that are not part of pairs
    paired_indices = set()
    for pair in sensor_pairs:
        paired_indices.update(pair)
    
    unpaired_indices = [i for i in range(len(sensor_columns)) if i not in paired_indices]
    
    print(f"Paired sensor indices: {sorted(paired_indices)}")
    print(f"Unpaired sensor indices: {unpaired_indices}")
    
    for i, row in sensor_data.iterrows():
        unique_count = 0
        
        # Count unpaired sensors
        for sensor_idx in unpaired_indices:
            if row.iloc[sensor_idx] == 'ILLUMINATED':
                unique_count += 1
        
        # Count paired sensors (only count once per pair if either is illuminated)
        for pair in sensor_pairs:
            sensor1_illuminated = row.iloc[pair[0]] == 'ILLUMINATED'
            sensor2_illuminated = row.iloc[pair[1]] == 'ILLUMINATED'
            
            if sensor1_illuminated or sensor2_illuminated:
                unique_count += 1
        
        unique_illuminated_counts[i] = unique_count
    
    print(f"Raw illuminated sensor counts range: {raw_illuminated_counts.min()} to {raw_illuminated_counts.max()}")
    print(f"Unique illuminated sensor counts range: {unique_illuminated_counts.min()} to {unique_illuminated_counts.max()}")
    
    return normalized_vectors, unique_illuminated_counts, raw_illuminated_counts

def create_distribution_statistics(unique_counts, raw_counts):
    """
    Create and display statistics comparing unique vs raw sensor counts.
    
    Args:
        unique_counts (np.ndarray): Unique illuminated sensor direction counts
        raw_counts (np.ndarray): Raw illuminated sensor counts
    
    Returns:
        dict: Statistics dictionary
    """
    # Create distribution statistics
    stats = {
        'unique': {
            'mean': np.mean(unique_counts),
            'std': np.std(unique_counts),
            'min': np.min(unique_counts),
            'max': np.max(unique_counts),
            'median': np.median(unique_counts)
        },
        'raw': {
            'mean': np.mean(raw_counts),
            'std': np.std(raw_counts),
            'min': np.min(raw_counts),
            'max': np.max(raw_counts),
            'median': np.median(raw_counts)
        }
    }
    
    # Print statistics
    print("\n" + "="*60)
    print("SENSOR ILLUMINATION STATISTICS")
    print("="*60)
    print(f"{'Metric':<20} {'Unique Directions':<20} {'Raw Count':<20}")
    print("-"*60)
    print(f"{'Mean':<20} {stats['unique']['mean']:<20.2f} {stats['raw']['mean']:<20.2f}")
    print(f"{'Std Dev':<20} {stats['unique']['std']:<20.2f} {stats['raw']['std']:<20.2f}")
    print(f"{'Minimum':<20} {stats['unique']['min']:<20.0f} {stats['raw']['min']:<20.0f}")
    print(f"{'Maximum':<20} {stats['unique']['max']:<20.0f} {stats['raw']['max']:<20.0f}")
    print(f"{'Median':<20} {stats['unique']['median']:<20.2f} {stats['raw']['median']:<20.2f}")
    
    # Distribution of unique counts
    unique_values, unique_counts_dist = np.unique(unique_counts, return_counts=True)
    print(f"\nDistribution of Unique Sensor Direction Counts:")
    print(f"{'Directions':<15} {'Frequency':<15} {'Percentage':<15}")
    print("-"*45)
    for val, count in zip(unique_values, unique_counts_dist):
        percentage = (count / len(unique_counts)) * 100
        print(f"{val:<15.0f} {count:<15,} {percentage:<15.1f}%")
    
    return stats

def create_comparison_histogram(unique_counts, raw_counts):
    """
    Create a comparison histogram of unique vs raw sensor counts.
    """
    fig, ax1 = plt.subplots(1, 1, figsize=(10, 6))
    
    # Unique counts histogram
    ax1.hist(unique_counts, bins=int(unique_counts.max() - unique_counts.min() + 1), 
             alpha=0.7, color='skyblue', edgecolor='black')
    ax1.set_xlabel('Number of Unique Sensor Directions Illuminated')
    ax1.set_ylabel('Frequency')
    ax1.set_title('Distribution of Unique Sensor Direction Counts')
    ax1.grid(True, alpha=0.3)
    
    # Add statistics text
    stats_text1 = f'Mean: {np.mean(unique_counts):.2f}\nStd: {np.std(unique_counts):.2f}\nMedian: {np.median(unique_counts):.2f}'
    ax1.text(0.02, 0.98, stats_text1, transform=ax1.transAxes, 
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    plt.tight_layout()
    return fig

def create_sphere_heatmap_unique(normalized_vectors, unique_counts, max_sensors, resolution=100, colormap='viridis', interpolation_method='rbf'):
    """
    Create an interactive 3D sphere heatmap visualization using unique sensor direction counts.
    
    Args:
        interpolation_method (str): 'rbf' for smooth RBF, 'linear' for linear, 'cubic' for cubic, 'nearest' for discrete
    """
    # Create the 3D plot
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Convert Cartesian coordinates to spherical
    x, y, z = normalized_vectors[:, 0], normalized_vectors[:, 1], normalized_vectors[:, 2]
    
    # Create a uniform sphere mesh for interpolation
    phi = np.linspace(0, 2*np.pi, resolution)  # azimuth
    theta = np.linspace(0, np.pi, resolution//2)  # polar angle
    phi_mesh, theta_mesh = np.meshgrid(phi, theta)
    
    # Convert sphere mesh to Cartesian coordinates
    x_mesh = np.sin(theta_mesh) * np.cos(phi_mesh)
    y_mesh = np.sin(theta_mesh) * np.sin(phi_mesh)
    z_mesh = np.cos(theta_mesh)
    
    # Interpolate the unique sensor counts onto the sphere mesh
    points = np.column_stack([x, y, z])
    mesh_points = np.column_stack([x_mesh.ravel(), y_mesh.ravel(), z_mesh.ravel()])
    
    print(f"Interpolating unique sensor direction data using {interpolation_method} method...")
    
    if interpolation_method == 'rbf':
        # Use Radial Basis Function for smooth interpolation
        from scipy.interpolate import Rbf
        
        # Create RBF interpolator - 'multiquadric' gives smooth results
        print("Creating RBF interpolator for smooth surface...")
        rbf_interp = Rbf(x, y, z, unique_counts, function='multiquadric', smooth=0.1)
        
        # Evaluate RBF on mesh points
        interpolated_values = rbf_interp(x_mesh.ravel(), y_mesh.ravel(), z_mesh.ravel())
        interpolated_values = interpolated_values.reshape(x_mesh.shape)
        
        # Ensure values stay within reasonable bounds
        interpolated_values = np.clip(interpolated_values, 0, max_sensors)
        
    else:  # nearest
        interpolated_values = griddata(points, unique_counts, mesh_points, 
                                     method='nearest', fill_value=0)
        interpolated_values = interpolated_values.reshape(x_mesh.shape)
    
    # Normalize interpolated values for colormap
    normalized_values = interpolated_values / max_sensors
    
    # Create the surface plot with the smooth heatmap
    # Fix matplotlib deprecation warning
    cmap = plt.get_cmap(colormap)
    surface = ax.plot_surface(x_mesh, y_mesh, z_mesh, 
                             facecolors=cmap(normalized_values),
                             alpha=0.9,
                             linewidth=0,
                             antialiased=True,
                             shade=True,
                             rcount=resolution//2,  # Control surface resolution
                             ccount=resolution)
    
    # Create a dummy scatter plot for the colorbar
    dummy_scatter = ax.scatter([], [], [], c=[], cmap=colormap, vmin=0, vmax=max_sensors)
    
    # Set labels and title
    ax.set_xlabel('Sun X', fontsize=12)
    ax.set_ylabel('Sun Y', fontsize=12)
    ax.set_zlabel('Sun Z', fontsize=12)
    ax.set_title('Satellite Sun Sensor Illumination Heatmap\n'
                 f'Smooth Continuous Surface ({interpolation_method.upper()} interpolation)', 
                 fontsize=14, pad=20)
    
    # Set equal scaling for all axes
    max_range = 1.2
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range, max_range])
    ax.set_box_aspect([1,1,1])
    
    # Create colorbar
    cbar = plt.colorbar(dummy_scatter, ax=ax, shrink=0.6, aspect=30, pad=0.1)
    cbar.set_label('Number of Unique Sensor Directions Illuminated', fontsize=12)
    
    # Add statistics text
    stats_text = f"""Statistics (Unique Directions):
    Total Sun Vectors: {len(normalized_vectors):,}
    Min Illuminated: {unique_counts.min()}
    Max Illuminated: {unique_counts.max()}
    Mean Illuminated: {unique_counts.mean():.1f}
    Interpolation: {interpolation_method.upper()}
    Resolution: {resolution}×{resolution//2}
    Sensor Pairs: Y+, Y-, Z+, Z-"""
    
    ax.text2D(0.02, 0.98, stats_text, transform=ax.transAxes, 
              verticalalignment='top', fontsize=10,
              bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    plt.tight_layout()
    return fig, ax

def main():
    """
    Main function to orchestrate the visualization pipeline with unique sensor direction counting.
    """
    # Set up command line argument parsing
    parser = argparse.ArgumentParser(description='Visualize satellite sun sensor data with unique direction accounting')
    parser.add_argument('csv_file', nargs='?', default='simulation_data.csv',
                       help='Path to the CSV file (default: simulation_data.csv)')
    parser.add_argument('--save', type=str, help='Save plot to file (e.g., plot.png)')
    parser.add_argument('--save-stats', type=str, help='Save statistics histogram (e.g., stats.png)')
    parser.add_argument('--dpi', type=int, default=300, help='DPI for saved plot (default: 300)')
    parser.add_argument('--colormap', type=str, default='viridis',
                       help='Colormap for heatmap (e.g., viridis, plasma, inferno, magma, hot, cool)')
    parser.add_argument('--interpolation', choices=['rbf', 'nearest'], 
                       default='nearest', help='Interpolation method: rbf (smooth) or nearest (discrete)')
    parser.add_argument('--resolution', type=int, default=150,
                       help='Resolution for surface heatmap (default: 150)')
    parser.add_argument('--show-stats', action='store_true',
                       help='Show comparison histogram of unique vs raw counts')
    
    args = parser.parse_args()
    
    # Check if file exists
    if not os.path.exists(args.csv_file):
        print(f"CSV file '{args.csv_file}' not found.")
        print("Please provide the correct path to your CSV file.")
        print("Usage: python script.py your_file.csv")
        sys.exit(1)
    
    print(f"Processing CSV file: {args.csv_file}")
    print(f"Colormap: {args.colormap}")
    print(f"Interpolation method: {args.interpolation}")
    print("Accounting for paired sensors: Y+[9-10], Y-[11-12], Z+[13-14], Z-[15-16]")
    print("-" * 70)
    
    # Step 1: Read the CSV file
    data = read_csv_data(args.csv_file)
    
    # Step 2: Process vectors and count unique illuminated sensor directions
    normalized_vectors, unique_counts, raw_counts = process_sun_vectors_and_sensors_unique(data)
    
    # Step 3: Create and display statistics
    stats = create_distribution_statistics(unique_counts, raw_counts)
    
    # Step 4: Create comparison histogram if requested
    if args.show_stats:
        comparison_fig = create_comparison_histogram(unique_counts, raw_counts)
        if args.save_stats:
            print(f"Saving statistics histogram to {args.save_stats}...")
            comparison_fig.savefig(args.save_stats, dpi=args.dpi, bbox_inches='tight')
        plt.show(block=False)
    
    # Step 5: Create the 3D heatmap visualization using unique counts
    max_unique_sensors = unique_counts.max()
    
    print("-" * 70)
    print(f"Creating 3D sphere heatmap using {args.interpolation} interpolation...")
    
    fig, ax = create_sphere_heatmap_unique(normalized_vectors, unique_counts, max_unique_sensors, 
                                          args.resolution, args.colormap, args.interpolation)
    
    # Save plot if requested
    if args.save:
        print(f"Saving sphere heatmap to {args.save}...")
        fig.savefig(args.save, dpi=args.dpi, bbox_inches='tight')
        print("Plot saved successfully!")
    
    # Show the interactive plot
    print("Displaying interactive sphere heatmap with unique sensor direction counts.")
    print("Close the plot window to exit.")
    plt.show()

if __name__ == "__main__":
    main()