#!/usr/bin/env python3
"""
Sun Vector Validation Script
Compares sun vectors from C++ output with astropy calculations
Calculates angular errors and provides statistics
"""

import numpy as np
import pandas as pd
from astropy.time import Time
from astropy.coordinates import get_sun, GCRS
from astropy import units as u
import matplotlib.pyplot as plt
from datetime import datetime
import argparse
import sys

class SunVectorValidator:
    def __init__(self, csv_file):
        self.csv_file = csv_file
        self.data = None
        self.errors = []
        
    def load_data(self):
        """Load CSV data with date and sun vector components"""
        try:
            # Try to load with headers
            self.data = pd.read_csv(self.csv_file)
            
            # Check if headers exist or if first row is data
            if self.data.columns[0].replace('.', '').replace('-', '').replace(' ', '').replace(':', '').isdigit():
                # First row is data, reload without headers
                self.data = pd.read_csv(self.csv_file, header=None, 
                                      names=['datetime', 'x', 'y', 'z'])
            else:
                # Ensure columns are named correctly
                self.data.columns = ['datetime', 'x', 'y', 'z']
                
            print(f"Loaded {len(self.data)} sun vector measurements")
            
            # Parse datetime - now includes full timestamp
            self.data['datetime'] = pd.to_datetime(self.data['datetime'])
            
            # Calculate time step from actual data
            if len(self.data) > 1:
                time_diff = self.data['datetime'].iloc[1] - self.data['datetime'].iloc[0]
                hours_between_samples = time_diff.total_seconds() / 3600
                print(f"Time step: {hours_between_samples:.1f} hours")
            else:
                print("Single measurement")
                
        except Exception as e:
            print(f"Error loading CSV file: {e}")
            sys.exit(1)
            
    def normalize_vector(self, vector):
        """Normalize a vector to unit length"""
        norm = np.linalg.norm(vector)
        if norm == 0:
            return vector
        return vector / norm
        
    def calculate_angular_error(self, vec1, vec2):
        """Calculate angular error between two vectors in degrees"""
        # Normalize vectors
        vec1_norm = self.normalize_vector(vec1)
        vec2_norm = self.normalize_vector(vec2)
        
        # Calculate dot product
        dot_product = np.dot(vec1_norm, vec2_norm)
        
        # Clamp to avoid numerical errors
        dot_product = np.clip(dot_product, -1.0, 1.0)
        
        # Calculate angle in radians then convert to degrees
        angle_rad = np.arccos(dot_product)
        angle_deg = np.degrees(angle_rad)
        
        return angle_deg
        
    def get_astropy_sun_vector(self, datetime):
        """Get sun vector in ECI frame using astropy"""
        # Create astropy time object
        t = Time(datetime)
        
        # Get sun position in GCRS (essentially ECI)
        sun = get_sun(t)
        sun_gcrs = sun.transform_to(GCRS(obstime=t))
        
        # Extract cartesian coordinates
        x = sun_gcrs.cartesian.x.to(u.m).value
        y = sun_gcrs.cartesian.y.to(u.m).value
        z = sun_gcrs.cartesian.z.to(u.m).value
        
        # Return normalized vector
        return self.normalize_vector(np.array([x, y, z]))
        
    def validate_vectors(self):
        """Compare all vectors and calculate errors"""
        print("\nValidating sun vectors...")
        
        self.errors = []
        
        for idx, row in self.data.iterrows():
            # Get C++ vector
            cpp_vector = np.array([row['x'], row['y'], row['z']])
            cpp_vector_norm = self.normalize_vector(cpp_vector)
            
            # Get astropy vector
            astropy_vector = self.get_astropy_sun_vector(row['datetime'])
            
            # Calculate error
            error_deg = self.calculate_angular_error(cpp_vector_norm, astropy_vector)
            self.errors.append(error_deg)
            
            # Print progress
            if (idx + 1) % 100 == 0:
                print(f"Processed {idx + 1}/{len(self.data)} vectors...")
                
        self.errors = np.array(self.errors)
        print(f"Validation complete. Processed {len(self.errors)} vectors.")
        
    def calculate_statistics(self):
        """Calculate and display error statistics"""
        print("\n" + "="*50)
        print("SUN VECTOR ERROR STATISTICS")
        print("="*50)
        
        stats = {
            'Mean Error': np.mean(self.errors),
            'Median Error': np.median(self.errors),
            'Standard Deviation': np.std(self.errors),
            'Min Error': np.min(self.errors),
            'Max Error': np.max(self.errors),
            '95th Percentile': np.percentile(self.errors, 95),
            '99th Percentile': np.percentile(self.errors, 99)
        }
        
        for key, value in stats.items():
            print(f"{key:.<25} {value:.6f} degrees")
            
        # Additional analysis
        print(f"\nVectors with error > 1 degree: {np.sum(self.errors > 1)}")
        print(f"Vectors with error > 0.1 degree: {np.sum(self.errors > 0.1)}")
        print(f"Vectors with error > 0.01 degree: {np.sum(self.errors > 0.01)}")
        
        return stats
        
    def plot_errors(self, save_fig=False):
        """Create visualization of errors"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Sun Vector Error Analysis', fontsize=16)
        
        # Time series plot
        ax1 = axes[0, 0]
        ax1.plot(self.data['datetime'], self.errors, 'b-', linewidth=0.5)
        ax1.set_xlabel('Date')
        ax1.set_ylabel('Angular Error (degrees)')
        ax1.set_title('Error Over Time')
        ax1.grid(True, alpha=0.3)
        
        # Format x-axis dates
        import matplotlib.dates as mdates
        ax1.xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m'))
        ax1.xaxis.set_major_locator(mdates.MonthLocator())
        plt.setp(ax1.xaxis.get_majorticklabels(), rotation=45)
        
        # Histogram
        ax2 = axes[0, 1]
        ax2.hist(self.errors, bins=50, color='green', alpha=0.7, edgecolor='black')
        ax2.set_xlabel('Angular Error (degrees)')
        ax2.set_ylabel('Frequency')
        ax2.set_title('Error Distribution')
        ax2.grid(True, alpha=0.3)
        
        # Log scale histogram
        ax3 = axes[1, 0]
        ax3.hist(self.errors, bins=50, color='red', alpha=0.7, edgecolor='black')
        ax3.set_xlabel('Angular Error (degrees)')
        ax3.set_ylabel('Frequency (log scale)')
        ax3.set_title('Error Distribution (Log Scale)')
        ax3.set_yscale('log')
        ax3.grid(True, alpha=0.3)
        
        # Cumulative distribution
        ax4 = axes[1, 1]
        sorted_errors = np.sort(self.errors)
        cumulative = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors) * 100
        ax4.plot(sorted_errors, cumulative, 'purple', linewidth=2)
        ax4.set_xlabel('Angular Error (degrees)')
        ax4.set_ylabel('Cumulative Percentage (%)')
        ax4.set_title('Cumulative Error Distribution')
        ax4.grid(True, alpha=0.3)
        
        # Add reference lines
        ax4.axhline(y=95, color='r', linestyle='--', alpha=0.5, label='95%')
        ax4.axhline(y=99, color='orange', linestyle='--', alpha=0.5, label='99%')
        ax4.legend()
        
        plt.tight_layout()
        
        if save_fig:
            filename = f"sun_vector_errors_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            plt.savefig(filename, dpi=300)
            print(f"\nPlot saved as: {filename}")
            
        plt.show()
        
    def save_detailed_results(self, output_file=None):
        """Save detailed results with errors for each measurement"""
        if output_file is None:
            output_file = f"sun_vector_validation_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            
        # Create results dataframe
        results = self.data.copy()
        results['angular_error_deg'] = self.errors
        
        # Calculate astropy vectors for comparison
        astropy_vectors = []
        for idx, row in self.data.iterrows():
            vec = self.get_astropy_sun_vector(row['datetime'])
            astropy_vectors.append(vec)
            
        astropy_vectors = np.array(astropy_vectors)
        results['astropy_x'] = astropy_vectors[:, 0]
        results['astropy_y'] = astropy_vectors[:, 1]
        results['astropy_z'] = astropy_vectors[:, 2]
        
        # Save to CSV
        results.to_csv(output_file, index=False)
        print(f"\nDetailed results saved to: {output_file}")

def main():
    parser = argparse.ArgumentParser(description="Validate sun vectors against astropy")
    parser.add_argument("csv_file", type=str, help="CSV file with time,x,y,z columns")
    parser.add_argument("--plot", action="store_true", help="Generate error plots")
    parser.add_argument("--save-plot", action="store_true", help="Save plot to file")
    parser.add_argument("--save-results", action="store_true", 
                      help="Save detailed validation results")
    
    args = parser.parse_args()
    
    # Create validator
    validator = SunVectorValidator(args.csv_file)
    
    # Load data
    validator.load_data()
    
    # Validate vectors
    validator.validate_vectors()
    
    # Calculate statistics
    validator.calculate_statistics()
    
    # Generate plots if requested
    if args.plot or args.save_plot:
        validator.plot_errors(save_fig=args.save_plot)
        
    # Save detailed results if requested
    if args.save_results:
        validator.save_detailed_results()

if __name__ == "__main__":
    main()