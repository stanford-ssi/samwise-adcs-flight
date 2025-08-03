# %%
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Set the CSV file name (update this to match your file)
csv_file = "magnetic_field_data.csv"  # Update with your actual filename

print("Simple Magnetic Field Contour Plotter")
print("=" * 40)

# Read the CSV file
df = pd.read_csv(csv_file)
print(f"Loaded {len(df)} data points")

# Clean up column names by stripping whitespace
df.columns = df.columns.str.strip()
print(f"Cleaned column names: {list(df.columns)}")

# Display the first few rows to verify structure
print("\nFirst 5 rows of data:")
print(df.head())
print(f"\nColumns: {list(df.columns)}")

# Get unique longitude and latitude values
unique_lons = np.sort(df['Longitude'].unique())
unique_lats = np.sort(df['Latitude'].unique())

print(f"Longitude points: {len(unique_lons)}")
print(f"Latitude points: {len(unique_lats)}")

# Check if altitude data is available
if 'Altitude' in df.columns:
    unique_alts = np.sort(df['Altitude'].unique())
    print(f"Altitude points: {len(unique_alts)}")

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
altitude = df['Altitude'].values.reshape(n_rows, n_lon)
b_r = df['B_r'].values.reshape(n_rows, n_lon)
b_phi = df['B_phi'].values.reshape(n_rows, n_lon)
b_theta = df['B_theta'].values.reshape(n_rows, n_lon)

# Use provided magnitude or calculate it
if 'B_magnitude' in df.columns:
    magnitude = df['B_magnitude'].values.reshape(n_rows, n_lon)
    print("Using provided B_magnitude values")
else:
    magnitude = np.sqrt(b_r**2 + b_phi**2 + b_theta**2)
    print("Calculated magnitude from components")

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
output_file = csv_file.replace('.csv', '_contours.png')
plt.savefig(output_file, dpi=300, bbox_inches='tight')
print(f"\nContour plots saved as: {output_file}")

# Show the plot
plt.show()

# %%
# Optional: 3D Vector Field Visualization
# Note: This section requires the magnetic field data to be on a 3D grid
# Uncomment and modify as needed for your specific use case

"""
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Create a coarser grid for vector visualization (for clarity)
step = max(1, len(unique_lons) // 10)  # Adjust step size as needed
lon_subset = longitude[::step, ::step]
lat_subset = latitude[::step, ::step] 
b_r_subset = b_r[::step, ::step]
b_phi_subset = b_phi[::step, ::step]
b_theta_subset = b_theta[::step, ::step]

# For 3D visualization, you can now use the altitude data
z_values = altitude[::step, ::step]  # Use actual altitude values

# Create 3D quiver plot
ax.quiver(lon_subset, lat_subset, z_values, 
          b_r_subset, b_phi_subset, b_theta_subset, 
          length=0.1, normalize=True, alpha=0.7)

ax.set_xlabel('Longitude')
ax.set_ylabel('Latitude') 
ax.set_zlabel('Altitude')
ax.set_title('3D Magnetic Field Vectors')

plt.show()
"""