# %%
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

 # Set the CSV file name (update this to match your file)
csv_file = "serial_data_20250607_135010.csv"  # Update with your actual filename

print("Simple Magnetic Field Contour Plotter")
print("=" * 40)


# Read the CSV file
df = pd.read_csv(csv_file)
print(f"Loaded {len(df)} data points")

# Get unique longitude and latitude values
unique_lons = np.sort(df['Longitude'].unique())
unique_lats = np.sort(df['Latitude'].unique())

print(f"Longitude points: {len(unique_lons)}")
print(f"Latitude points: {len(unique_lats)}")

# Determine grid dimensions
n_lon = len(unique_lons)
n_lat = len(unique_lats)
expected_points = n_lon * n_lat

print(f"Expected grid size: {n_lon} x {n_lat} = {expected_points} points")
print(f"Actual data points: {len(df)}")

# Truncate if necessary to make it square/rectangular
n_points_to_use = (len(df) // n_lon) * n_lon
if n_points_to_use < len(df):
    print(f"Truncating to {n_points_to_use} points to fit grid")
    df = df.iloc[:n_points_to_use].copy()

# Recalculate grid size after truncation
n_rows = len(df) // n_lon

# Reshape data into grids
longitude = df['Longitude'].values.reshape(n_rows, n_lon)
latitude = df['Latitude'].values.reshape(n_rows, n_lon)
b_r = df['B_r'].values.reshape(n_rows, n_lon)
b_phi = df['B_phi'].values.reshape(n_rows, n_lon)
b_theta = df['B_theta'].values.reshape(n_rows, n_lon)

# Calculate magnitude
magnitude = np.sqrt(b_r**2 + b_phi**2 + b_theta**2)

print(f"Final grid shape: {n_rows} x {n_lon}")

# Create figure with subplots
fig, axes = plt.subplots(2, 2, figsize=(15, 12))
fig.suptitle('Magnetic Field Components - Contour Plots', fontsize=16, fontweight='bold')

# Define the data and titles for each subplot
data_sets = [
    (b_r, 'B_r (Radial Component)', 'RdBu_r'),
    (b_phi, 'B_phi (Azimuthal Component)', 'RdYlBu_r'), 
    (b_theta, 'B_theta (Polar Component)', 'RdYlGn_r'),
    (magnitude, 'Magnetic Field Magnitude', 'plasma')
]

# Create contour plots for each component
for i, (data, title, colormap) in enumerate(data_sets):
    data = np.array(data)
    row = i // 2
    col = i % 2
    ax = axes[row, col]
    
    # Create contour plot
    contour_filled = ax.contourf(longitude, latitude, data, levels=20, cmap=colormap)
    contour_lines = ax.contour(longitude, latitude, data, levels=10, colors='black', alpha=0.4, linewidths=0.5)
    
    # Add some contour labels
    ax.clabel(contour_lines, inline=True, fontsize=8, fmt='%.0f')
    
    # Customize plot
    ax.set_xlabel('Longitude (degrees)', fontsize=12)
    ax.set_ylabel('Latitude (degrees)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    
    # Add colorbar
    cbar = plt.colorbar(contour_filled, ax=ax)
    cbar.set_label('Field Strength', fontsize=10)
    
    # Print statistics
    print(f"\n{title}:")
    print(f"  Min: {np.nanmin(data):.2f}")
    print(f"  Max: {np.nanmax(data):.2f}")
    print(f"  Mean: {np.nanmean(data):.2f}")

# Adjust layout
plt.tight_layout()

# Save the plot
output_file = csv_file.replace('.csv', '_simple_contours.png')
plt.savefig(output_file, dpi=300, bbox_inches='tight')
print(f"\nContour plots saved as: {output_file}")

# Show the plot
plt.show()


# %%
fig = plt.figure()
ax = fig.gca(projection='3d')

x, y, z = np.meshgrid(np.arange(-0.8, 1, 0.2),
                      np.arange(-0.8, 1, 0.2),
                      np.arange(-0.8, 1, 0.8))

ax.quiver(x, y, z, u, v, w, length=0.1)

plt.show()

# %%
