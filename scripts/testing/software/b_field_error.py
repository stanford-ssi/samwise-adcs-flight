import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from datetime import datetime
import cartopy.crs as ccrs
import cartopy.feature as cfeature

# IGRF package: pip install ppigrf
import ppigrf

def load_your_data(csv_file):
    """Load your magnetic field data"""
    df = pd.read_csv(csv_file)
    return df

def get_reference_igrf_data(lats, lons, altitude_km, date=None):
    """Get reference IGRF data using ppigrf package"""
    if date is None:
        date = datetime(2024, 1, 1)  # Default date
    
    print("Using ppigrf package for reference data...")
    ref_data = []
    
    for lat, lon in zip(lats, lons):
        try:
            # Check bounds like your example
            if not -90 < lat < 90:
                print(f"Skipping lat={lat} (out of bounds)")
                continue
            if not -180 <= lon <= 180:
                print(f"Skipping lon={lon} (out of bounds)")
                continue
            
            # ppigrf.igrf(lon, lat, alt_km, date) - using your exact interface
            # Returns Be (east), Bn (north), Bu (up) in nT
            Be, Bn, Bu = ppigrf.igrf(lon, lat, altitude_km, date)
            
            # Convert to scalars to avoid numpy deprecation warning
            B_east = float(Be.item() if hasattr(Be, 'item') else Be)    # nT
            B_north = float(Bn.item() if hasattr(Bn, 'item') else Bn)   # nT  
            B_up = float(Bu.item() if hasattr(Bu, 'item') else Bu)      # nT
            B_down = -B_up        # Convert up to down
            
            # Calculate total magnitude
            B_total = np.sqrt(B_east**2 + B_north**2 + B_up**2)
            
            # Convert NED to your spherical coordinates (r, phi, theta)
            B_r = B_up           # radial component (outward positive)
            B_phi = B_east       # azimuthal component
            B_theta = -B_north   # polar component (southward positive)
            
            ref_data.append({
                'Latitude': lat,
                'Longitude': lon,
                'Altitude': altitude_km,
                'B_r': B_r,
                'B_phi': B_phi,
                'B_theta': B_theta,
                'B_magnitude': B_total
            })
        except Exception as e:
            print(f"Error at lat={lat}, lon={lon}: {e}")
    
    return pd.DataFrame(ref_data)

def calculate_percent_error(your_data, ref_data):
    """Calculate percent error between your model and reference"""
    
    # Merge datasets on lat/lon
    merged = pd.merge(your_data, ref_data, on=['Latitude', 'Longitude'], 
                      suffixes=('_yours', '_ref'))
    
    # Calculate percent errors
    components = ['B_r', 'B_phi', 'B_theta', 'B_magnitude']
    
    for comp in components:
        your_col = f"{comp}_yours"
        ref_col = f"{comp}_ref"
        error_col = f"{comp}_error_percent"
        
        # Percent error: (your_value - ref_value) / ref_value * 100
        merged[error_col] = ((merged[your_col] - merged[ref_col]) / merged[ref_col]) * 100
    
    return merged

def create_geographic_error_maps(error_data, save_plots=True):
    """Create geographic contour plots of error maps"""
    
    components = ['B_r', 'B_phi', 'B_theta', 'B_magnitude']
    component_names = ['B_r (Radial)', 'B_φ (Azimuthal)', 'B_θ (Polar)', 'B Magnitude']
    
    # Check if data is on regular grid for contouring
    unique_lats = np.sort(error_data['Latitude'].unique())
    unique_lons = np.sort(error_data['Longitude'].unique())
    
    fig = plt.figure(figsize=(16, 12))
    
    for i, (comp, name) in enumerate(zip(components, component_names)):
        ax = plt.subplot(2, 2, i+1, projection=ccrs.PlateCarree())
        
        # Add map features
        ax.add_feature(cfeature.COASTLINE, linewidth=0.5)
        ax.add_feature(cfeature.BORDERS, linewidth=0.3)
        ax.add_feature(cfeature.OCEAN, color='lightblue', alpha=0.3)
        ax.add_feature(cfeature.LAND, color='lightgray', alpha=0.3)
        
        error_col = f"{comp}_error_percent"
        
        # Create contour plot if regular grid, scatter if not
        if len(unique_lats) * len(unique_lons) == len(error_data):
            # Regular grid - create contour plot
            error_data_sorted = error_data.sort_values(['Latitude', 'Longitude'])
            error_grid = error_data_sorted[error_col].values.reshape(len(unique_lats), len(unique_lons))
            
            lon_mesh, lat_mesh = np.meshgrid(unique_lons, unique_lats)
            
            # Set symmetric color limits
            vmax = np.percentile(np.abs(error_grid[~np.isnan(error_grid)]), 95)
            levels = np.linspace(-vmax, vmax, 20)
            
            # Create filled contour plot
            contour = ax.contourf(lon_mesh, lat_mesh, error_grid, levels=levels,
                                 cmap='RdBu_r', transform=ccrs.PlateCarree(), alpha=0.8)
            
            # Add contour lines
            contour_lines = ax.contour(lon_mesh, lat_mesh, error_grid, levels=levels[::2],
                                      colors='black', linewidths=0.5, alpha=0.6,
                                      transform=ccrs.PlateCarree())
        else:
            # Fallback to scatter plot
            vmax = np.percentile(np.abs(error_data[error_col].dropna()), 95)
            contour = ax.scatter(error_data['Longitude'], error_data['Latitude'], 
                               c=error_data[error_col], cmap='RdBu_r', s=20,
                               vmin=-vmax, vmax=vmax, alpha=0.8,
                               transform=ccrs.PlateCarree())
        
        # Add colorbar
        cbar = plt.colorbar(contour, ax=ax, orientation='horizontal', 
                           pad=0.05, aspect=50, shrink=0.8)
        cbar.set_label('Error (%)')
        
        # Set extent and features
        ax.set_global()
        ax.gridlines(draw_labels=True, alpha=0.5, linestyle='--')
        ax.set_title(f'{name} - Error Contours')
    
    plt.tight_layout()
    plt.suptitle('IGRF Model Comparison - Error Contour Maps', fontsize=16, y=0.98)
    
    if save_plots:
        plt.savefig('igrf_geographic_error_maps.png', dpi=300, bbox_inches='tight')
        print("Saved: igrf_geographic_error_maps.png")
    
    plt.show()

def create_magnetic_direction_error_map(error_data, save_plots=True):
    """Create geographic contour map showing error in magnetic field direction"""
    
    # Calculate unit vectors for your model and reference
    your_B_mag = np.sqrt(error_data['B_r_yours']**2 + error_data['B_phi_yours']**2 + error_data['B_theta_yours']**2)
    ref_B_mag = np.sqrt(error_data['B_r_ref']**2 + error_data['B_phi_ref']**2 + error_data['B_theta_ref']**2)
    
    # Unit vectors for your model
    your_unit_r = error_data['B_r_yours'] / your_B_mag
    your_unit_phi = error_data['B_phi_yours'] / your_B_mag
    your_unit_theta = error_data['B_theta_yours'] / your_B_mag
    
    # Unit vectors for reference model
    ref_unit_r = error_data['B_r_ref'] / ref_B_mag
    ref_unit_phi = error_data['B_phi_ref'] / ref_B_mag
    ref_unit_theta = error_data['B_theta_ref'] / ref_B_mag
    
    # Calculate dot product between unit vectors
    dot_product = (your_unit_r * ref_unit_r + 
                   your_unit_phi * ref_unit_phi + 
                   your_unit_theta * ref_unit_theta)
    
    # Clamp to [-1, 1] to handle numerical errors
    dot_product = np.clip(dot_product, -1.0, 1.0)
    
    # Calculate angular error in degrees
    angular_error_rad = np.arccos(np.abs(dot_product))
    angular_error_deg = np.degrees(angular_error_rad)
    
    # Add angular error to the error_data DataFrame
    error_data['angular_error_deg'] = angular_error_deg
    
    # Check if regular grid for contouring
    unique_lats = np.sort(error_data['Latitude'].unique())
    unique_lons = np.sort(error_data['Longitude'].unique())
    
    # Create the contour plot
    fig = plt.figure(figsize=(14, 8))
    ax = plt.subplot(1, 1, 1, projection=ccrs.PlateCarree())
    
    # Add map features
    ax.add_feature(cfeature.COASTLINE, linewidth=0.5)
    ax.add_feature(cfeature.BORDERS, linewidth=0.3)
    ax.add_feature(cfeature.OCEAN, color='lightblue', alpha=0.3)
    ax.add_feature(cfeature.LAND, color='lightgray', alpha=0.3)
    
    if len(unique_lats) * len(unique_lons) == len(error_data):
        # Regular grid - create contour plot
        print(f"Creating contour plot with {len(unique_lats)} x {len(unique_lons)} grid")
        error_data_sorted = error_data.sort_values(['Latitude', 'Longitude'])
        angular_grid = error_data_sorted['angular_error_deg'].values.reshape(len(unique_lats), len(unique_lons))
        
        lon_mesh, lat_mesh = np.meshgrid(unique_lons, unique_lats)
        
        # Create contour levels
        vmin = np.nanmin(angular_grid)
        vmax = np.nanmax(angular_grid)
        levels = np.linspace(vmin, vmax, 25)
        
        # Create filled contour plot
        contour = ax.contourf(lon_mesh, lat_mesh, angular_grid, levels=levels,
                             cmap='plasma', transform=ccrs.PlateCarree(), alpha=0.8, extend='both')
        
        # Add contour lines for better definition
        contour_lines = ax.contour(lon_mesh, lat_mesh, angular_grid, levels=levels[::3],
                                  colors='white', linewidths=0.5, alpha=0.7,
                                  transform=ccrs.PlateCarree())
        
        # Add contour labels
        ax.clabel(contour_lines, inline=True, fontsize=8, fmt='%.2f°')
        
    else:
        # Fallback to scatter plot if not regular grid
        print("Data not on regular grid, using scatter plot")
        vmax = np.percentile(angular_error_deg.dropna(), 95)
        contour = ax.scatter(error_data['Longitude'], error_data['Latitude'], 
                           c=angular_error_deg, cmap='plasma', s=25,
                           vmin=0, vmax=vmax, alpha=0.8,
                           transform=ccrs.PlateCarree())
    
    # Add colorbar
    cbar = plt.colorbar(contour, ax=ax, orientation='horizontal', 
                       pad=0.05, aspect=50, shrink=0.8)
    cbar.set_label('Angular Error (degrees)', fontsize=12)
    
    # Set extent and features
    ax.set_global()
    gl = ax.gridlines(draw_labels=True, alpha=0.5, linestyle='--')
    gl.top_labels = False
    gl.right_labels = False
    ax.set_title('Magnetic Field Direction Error', fontsize=14, pad=20)
    
    # Add statistics text
    mean_error = np.mean(angular_error_deg.dropna())
    max_error = np.max(angular_error_deg.dropna())
    min_error = np.min(angular_error_deg.dropna())
    p95_error = np.percentile(angular_error_deg.dropna(), 95)
    
    stats_text = f'Min: {min_error:.3f}°  Mean: {mean_error:.3f}°  Max: {max_error:.3f}°  95th: {p95_error:.3f}°'
    ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.9),
            verticalalignment='top', fontsize=11, weight='bold')
    
    plt.tight_layout()
    
    if save_plots:
        plt.savefig('magnetic_direction_error_contours.png', dpi=300, bbox_inches='tight')
        print("Saved: magnetic_direction_error_contours.png")
    
    plt.show()
    
    # Print direction error statistics
    print(f"\nMagnetic Field Direction Error Statistics:")
    print(f"  Mean Angular Error:    {mean_error:.4f}°")
    print(f"  RMS Angular Error:     {np.sqrt(np.mean(angular_error_deg**2)):.4f}°")
    print(f"  Min Angular Error:     {min_error:.4f}°")
    print(f"  Max Angular Error:     {max_error:.4f}°")
    print(f"  95th Percentile:       {p95_error:.4f}°")
    print(f"  Median Error:          {np.median(angular_error_deg.dropna()):.4f}°")
    
    return angular_error_deg

def print_error_statistics(error_data):
    """Print comprehensive error statistics"""
    
    components = ['B_r', 'B_phi', 'B_theta', 'B_magnitude']
    component_names = ['B_r (Radial)', 'B_φ (Azimuthal)', 'B_θ (Polar)', 'B Magnitude']
    
    print("\n" + "="*60)
    print("IGRF MODEL COMPARISON - ERROR STATISTICS")
    print("="*60)
    
    for comp, name in zip(components, component_names):
        error_col = f"{comp}_error_percent"
        errors = error_data[error_col].dropna()
        
        print(f"\n{name}:")
        print(f"  Mean Error:        {errors.mean():.3f}%")
        print(f"  RMS Error:         {np.sqrt(np.mean(errors**2)):.3f}%")
        print(f"  Std Deviation:     {errors.std():.3f}%")
        print(f"  Max Error:         {errors.max():.3f}%")
        print(f"  Min Error:         {errors.min():.3f}%")
        print(f"  95th Percentile:   {np.percentile(np.abs(errors), 95):.3f}%")
        print(f"  Median Abs Error:  {np.median(np.abs(errors)):.3f}%")

def compare_igrf_models(csv_file, date=None, save_plots=True):
    """Main function to compare IGRF models"""
    
    print("Loading your magnetic field data...")
    your_data = load_your_data(csv_file)
    
    print(f"Loaded {len(your_data)} data points")
    
    # Get altitude (assuming constant)
    altitude = your_data['Altitude'].iloc[0]
    print(f"Altitude: {altitude} km")
    
    # Get reference IGRF data
    print("Computing reference IGRF data...")
    ref_data = get_reference_igrf_data(your_data['Latitude'], 
                                       your_data['Longitude'], 
                                       altitude, date)
    
    print(f"Generated {len(ref_data)} reference points")
    
    # Calculate errors
    print("Calculating percent errors...")
    error_data = calculate_percent_error(your_data, ref_data)
    
    # Print statistics
    print_error_statistics(error_data)
    
    # Create plot
    print("Creating magnetic direction error contour map...")
    angular_errors = create_magnetic_direction_error_map(error_data, save_plots)
    
    # Save error data
    if save_plots:
        error_data.to_csv('igrf_comparison_results.csv', index=False)
        print("Saved: igrf_comparison_results.csv")
    
    return error_data

# Main execution
if __name__ == "__main__":
    csv_file = 'magnetic_data.csv'
    
    # Set the date for IGRF model (adjust as needed)
    comparison_date = datetime(2024, 1, 1)
    
    # Run comparison
    error_results = compare_igrf_models(csv_file, date=comparison_date, save_plots=True)
    
    print("\nComparison complete!")
    print("Generated files:")
    print("  - magnetic_direction_error_contours.png") 
    print("  - igrf_comparison_results.csv (includes angular_error_deg column)")