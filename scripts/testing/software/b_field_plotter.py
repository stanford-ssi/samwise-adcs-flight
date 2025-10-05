import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.interpolate import griddata
import cartopy.crs as ccrs
import cartopy.feature as cfeature

def load_and_prepare_data(csv_file):
    """Load CSV data and prepare for plotting"""
    df = pd.read_csv(csv_file)
    return df

def create_magnetic_field_plots(csv_file):
    """Create contour plots for magnetic field components on world maps"""
    
    # Load data
    df = load_and_prepare_data(csv_file)
    
    # Extract coordinates and magnetic field components
    lons = df['Longitude'].values
    lats = df['Latitude'].values
    br = df['B_r'].values
    btheta = df['B_theta'].values
    bphi = df['B_phi'].values
    bmag = df['B_magnitude'].values
    
    print(f"Loaded {len(df)} data points")
    print(f"Lat range: {lats.min():.1f} to {lats.max():.1f}°")
    print(f"Lon range: {lons.min():.1f} to {lons.max():.1f}°")
    
    # Check if data is on a regular grid (from your C mapping function)
    unique_lats = np.unique(lats)
    unique_lons = np.unique(lons)
    
    if len(unique_lats) * len(unique_lons) == len(df):
        print("Regular grid detected - using direct reshape")
        # Data is on regular grid, reshape directly
        n_lat = len(unique_lats)
        n_lon = len(unique_lons)
        
        # Sort data to ensure proper ordering
        df_sorted = df.sort_values(['Latitude', 'Longitude'])
        
        # Reshape to grid format
        lat_mesh, lon_mesh = np.meshgrid(unique_lats, unique_lons, indexing='ij')
        br_grid = df_sorted['B_r'].values.reshape(n_lat, n_lon)
        btheta_grid = df_sorted['B_theta'].values.reshape(n_lat, n_lon)
        bphi_grid = df_sorted['B_phi'].values.reshape(n_lat, n_lon)
        bmag_grid = df_sorted['B_magnitude'].values.reshape(n_lat, n_lon)
        
    else:
        print("Irregular grid - using interpolation")
        # Create regular grid for interpolation
        lon_grid = np.linspace(lons.min(), lons.max(), 180)
        lat_grid = np.linspace(lats.min(), lats.max(), 90)
        lon_mesh, lat_mesh = np.meshgrid(lon_grid, lat_grid)
        
        # Interpolate data onto grid
        br_grid = griddata((lons, lats), br, (lon_mesh, lat_mesh), method='cubic', fill_value=np.nan)
        btheta_grid = griddata((lons, lats), btheta, (lon_mesh, lat_mesh), method='cubic', fill_value=np.nan)
        bphi_grid = griddata((lons, lats), bphi, (lon_mesh, lat_mesh), method='cubic', fill_value=np.nan)
        bmag_grid = griddata((lons, lats), bmag, (lon_mesh, lat_mesh), method='cubic', fill_value=np.nan)
    
    # Create subplots
    fig = plt.figure(figsize=(16, 12))
    
    # Magnetic field components and labels
    components = [
        (br_grid, br, 'B_r (nT)', 'Radial Component'),
        (btheta_grid, btheta, 'B_θ (nT)', 'Theta Component'),
        (bphi_grid, bphi, 'B_φ (nT)', 'Phi Component'),
        (bmag_grid, bmag, 'B_magnitude (nT)', 'Magnitude')
    ]
    
    for i, (grid_data, point_data, label, title) in enumerate(components, 1):
        ax = plt.subplot(2, 2, i, projection=ccrs.PlateCarree())
        
        # Add map features
        ax.add_feature(cfeature.COASTLINE, linewidth=0.5)
        ax.add_feature(cfeature.BORDERS, linewidth=0.5)
        ax.add_feature(cfeature.OCEAN, color='lightblue', alpha=0.5)
        ax.add_feature(cfeature.LAND, color='lightgray', alpha=0.5)
        
        # Create contour plot - use global extent for your grid data
        ax.set_global()
        
        # Use more contour levels for better resolution with your dense grid
        levels = 30
        contour = ax.contourf(lon_mesh, lat_mesh, grid_data, levels=levels, 
                             cmap='RdYlBu_r', transform=ccrs.PlateCarree(), alpha=0.8)
        
        # Add contour lines for better visualization
        contour_lines = ax.contour(lon_mesh, lat_mesh, grid_data, levels=levels//3, 
                                  colors='black', linewidths=0.5, alpha=0.6,
                                  transform=ccrs.PlateCarree())
        
        # Plot sample data points (subsample for clarity on dense grid)
        step = max(1, len(df) // 200)  # Show max 200 points for clarity
        sample_indices = np.arange(0, len(df), step)
        scatter = ax.scatter(lons[sample_indices], lats[sample_indices], 
                           c=point_data[sample_indices], s=20, 
                           cmap='RdYlBu_r', edgecolors='black', linewidth=0.5,
                           transform=ccrs.PlateCarree(), zorder=5, alpha=0.8)
        
        # Add colorbar
        cbar = plt.colorbar(contour, ax=ax, orientation='horizontal', 
                           pad=0.05, aspect=50, shrink=0.8)
        cbar.set_label(label)
        
        # Add gridlines
        ax.gridlines(draw_labels=True, dms=True, x_inline=False, y_inline=False, 
                    alpha=0.5, linestyle='--')
        
        # Set title
        ax.set_title(f'{title}\nAltitude: {df["Altitude"].iloc[0]} km')
    
    plt.tight_layout()
    plt.suptitle('Magnetic Field Components - World Map Contour Plots', 
                 fontsize=16, y=0.98)
    
    # Save the plot
    plt.savefig('magnetic_field_contours.png', dpi=300, bbox_inches='tight')
    print("Saved: magnetic_field_contours.png")
    plt.show()
    
if __name__ == "__main__":
    csv_file = 'magnetic_data.csv'
    
    # Create contour plots
    create_magnetic_field_plots(csv_file)