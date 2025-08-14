import numpy as np
import serial
import re
import time
import json
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
from datetime import datetime

class MagnetometerCalibrator:
    def __init__(self, port='/dev/tty.usbmodem1101', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.raw_data = []
        self.calibration_params = None
        self.calibration_quality = None
        
    def collect_data(self, duration_minutes=10, min_samples=1000, reconnect_delay=2):
        """Collect magnetometer data over serial for calibration with auto-reconnect"""
        print(f"Collecting magnetometer data for {duration_minutes} minutes...")
        print("Rotate the satellite/magnetometer in all orientations during this time!")
        print("Auto-reconnect enabled for watchdog reboots...")
        
        start_time = time.time()
        sample_count = 0
        ser = None
        reconnect_count = 0
        pattern = r'Magnetometer reading: \[([-\d.]+), ([-\d.]+), ([-\d.]+)\]'
        
        while (time.time() - start_time) < (duration_minutes * 60) and sample_count < min_samples * 2:
            try:
                # Try to establish/re-establish connection
                if ser is None or not ser.is_open:
                    if ser:
                        try:
                            ser.close()
                        except:
                            pass
                    
                    print(f"{'Connecting' if reconnect_count == 0 else 'Reconnecting'} to {self.port}...")
                    ser = serial.Serial(self.port, self.baudrate, timeout=1)
                    
                    if reconnect_count > 0:
                        print(f"Reconnection #{reconnect_count} successful!")
                        # Give the satellite a moment to boot up
                        time.sleep(1)
                    
                    reconnect_count += 1
                
                # Try to read data
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                match = re.search(pattern, line)
                if match:
                    x, y, z = map(float, match.groups())
                    self.raw_data.append([x, y, z])
                    sample_count += 1
                    
                    if sample_count % 100 == 0:
                        elapsed = time.time() - start_time
                        rate = sample_count / elapsed if elapsed > 0 else 0
                        print(f"Collected {sample_count} samples... ({rate:.1f} samples/sec)")
                
            except (serial.SerialException, OSError) as e:
                # Serial connection issues - likely watchdog reboot
                print(f"Connection lost (likely watchdog reboot): {e}")
                if ser and ser.is_open:
                    try:
                        ser.close()
                    except:
                        pass
                ser = None
                
                print(f"Waiting {reconnect_delay}s before reconnection attempt...")
                time.sleep(reconnect_delay)
                continue
                
            except Exception as e:
                # Other errors - continue trying
                print(f"Read error: {e}")
                time.sleep(0.1)
                continue
        
        # Clean up
        if ser and ser.is_open:
            try:
                ser.close()
            except:
                pass
        
        print(f"Collection complete! Got {len(self.raw_data)} samples")
        print(f"Total reconnections: {reconnect_count - 1}")
        
        return len(self.raw_data) >= min_samples
    
    def load_data_from_file(self, filename):
        """Load previously collected data from file"""
        with open(filename, 'r') as f:
            self.raw_data = json.load(f)
        print(f"Loaded {len(self.raw_data)} samples from {filename}")
    
    def save_data_to_file(self, filename):
        """Save collected data to file"""
        with open(filename, 'w') as f:
            json.dump(self.raw_data, f)
        print(f"Saved {len(self.raw_data)} samples to {filename}")
    
    def sphere_fit(self, data):
        """Fit a sphere to the magnetometer data using least squares"""
        def residuals(params):
            cx, cy, cz, r = params
            distances = np.sqrt((data[:, 0] - cx)**2 + (data[:, 1] - cy)**2 + (data[:, 2] - cz)**2)
            return distances - r
        
        # Initial guess: center at mean, radius as std
        center_guess = np.mean(data, axis=0)
        radius_guess = np.std(data)
        initial_params = [center_guess[0], center_guess[1], center_guess[2], radius_guess]
        
        # Minimize residuals
        result = minimize(lambda p: np.sum(residuals(p)**2), initial_params, method='BFGS')
        
        if result.success:
            cx, cy, cz, r = result.x
            return {'center': [cx, cy, cz], 'radius': r}
        else:
            raise Exception("Sphere fitting failed")
    
    def ellipsoid_fit(self, data):
        """Advanced calibration: fit ellipsoid to correct for soft iron effects"""
        # Center the data
        center = np.mean(data, axis=0)
        centered_data = data - center
        
        # Build design matrix for ellipsoid fitting
        # Equation: ax^2 + by^2 + cz^2 + 2dxy + 2exz + 2fyz = 1
        D = np.column_stack([
            centered_data[:, 0]**2,
            centered_data[:, 1]**2, 
            centered_data[:, 2]**2,
            2 * centered_data[:, 0] * centered_data[:, 1],
            2 * centered_data[:, 0] * centered_data[:, 2],
            2 * centered_data[:, 1] * centered_data[:, 2]
        ])
        
        # Solve using least squares
        ones = np.ones(D.shape[0])
        coeffs = np.linalg.lstsq(D, ones, rcond=None)[0]
        
        # Convert to matrix form
        A = np.array([
            [coeffs[0], coeffs[3], coeffs[4]],
            [coeffs[3], coeffs[1], coeffs[5]],
            [coeffs[4], coeffs[5], coeffs[2]]
        ])
        
        # Eigenvalue decomposition for transformation matrix
        eigenvals, eigenvecs = np.linalg.eig(A)
        
        # Ensure positive eigenvalues
        if np.any(eigenvals <= 0):
            eigenvals = np.abs(eigenvals)
        
        # Compute transformation matrix
        D_sqrt = np.diag(np.sqrt(eigenvals))
        transform_matrix = eigenvecs @ D_sqrt @ eigenvecs.T
        
        return {
            'center': center,
            'transform_matrix': transform_matrix,
            'scale_factors': 1.0 / np.sqrt(eigenvals),
            'A_matrix': A
        }
    
    def calibrate(self, method='sphere'):
        """Perform calibration using specified method"""
        if len(self.raw_data) < 500:
            raise Exception("Need at least 500 samples for calibration")
        
        data = np.array(self.raw_data)
        
        if method == 'sphere':
            self.calibration_params = self.sphere_fit(data)
            self.calibration_params['method'] = 'sphere'
        elif method == 'ellipsoid':
            self.calibration_params = self.ellipsoid_fit(data)
            self.calibration_params['method'] = 'ellipsoid'
        else:
            raise ValueError("Method must be 'sphere' or 'ellipsoid'")
        
        print(f"Calibration complete using {method} method")
        
        # Calculate calibration quality
        self._evaluate_calibration_quality()
        
        return self.calibration_params
    
    def _evaluate_calibration_quality(self):
        """Evaluate the quality of the calibration"""
        if self.calibration_params is None:
            return
        
        # Apply calibration to all data points
        calibrated_data = np.array([self.apply_calibration(point) for point in self.raw_data])
        
        # Compute distance from origin for calibrated data
        distances = np.linalg.norm(calibrated_data, axis=1)
        
        # Ideal case: all distances should be equal (perfect sphere)
        mean_distance = np.mean(distances)
        std_distance = np.std(distances)
        
        # Quality metric: lower std deviation relative to mean is better
        quality_score = std_distance / mean_distance
        
        self.calibration_quality = {
            'mean_radius': mean_distance,
            'std_radius': std_distance,
            'quality_score': quality_score,
            'sphericity': 1.0 - quality_score  # Higher is better (max 1.0 for perfect sphere)
        }
        
        print(f"\nCalibration Quality Assessment:")
        print(f"  Mean radius: {mean_distance:.3f} µT")
        print(f"  Radius std dev: {std_distance:.3f} µT")
        print(f"  Quality score: {quality_score:.4f} (lower is better)")
        print(f"  Sphericity: {self.calibration_quality['sphericity']:.4f} (higher is better)")
        
        if quality_score < 0.05:
            print("  ✓ Excellent calibration!")
        elif quality_score < 0.1:
            print("  ✓ Good calibration")
        elif quality_score < 0.2:
            print("  ⚠ Fair calibration - consider collecting more data")
        else:
            print("  ⚠ Poor calibration - more data needed or check sensor mounting")
    
    def apply_calibration(self, raw_reading):
        """Apply calibration to a raw magnetometer reading"""
        if self.calibration_params is None:
            raise Exception("No calibration parameters available")
        
        raw = np.array(raw_reading)
        
        if self.calibration_params['method'] == 'sphere':
            # Simple offset correction
            center = np.array(self.calibration_params['center'])
            calibrated = raw - center
            
        elif self.calibration_params['method'] == 'ellipsoid':
            # Full ellipsoid correction
            center = self.calibration_params['center']
            transform = self.calibration_params['transform_matrix']
            
            # Apply transformation
            centered = raw - center
            calibrated = transform @ centered
        
        return calibrated.tolist()
    
    def save_calibration(self, filename):
        """Save calibration parameters to file"""
        if self.calibration_params is None:
            raise Exception("No calibration to save")
        
        # Convert numpy arrays to lists for JSON serialization
        params_to_save = {}
        for key, value in self.calibration_params.items():
            if isinstance(value, np.ndarray):
                params_to_save[key] = value.tolist()
            else:
                params_to_save[key] = value
        
        # Add quality metrics
        if self.calibration_quality is not None:
            params_to_save['calibration_quality'] = self.calibration_quality
        
        with open(filename, 'w') as f:
            json.dump(params_to_save, f, indent=2)
        print(f"Calibration saved to {filename}")
    
    def load_calibration(self, filename):
        """Load calibration parameters from file"""
        with open(filename, 'r') as f:
            params = json.load(f)
        
        # Convert lists back to numpy arrays where needed
        for key, value in params.items():
            if key in ['center', 'transform_matrix', 'scale_factors', 'A_matrix'] and isinstance(value, list):
                params[key] = np.array(value)
        
        # Extract quality metrics if present
        if 'calibration_quality' in params:
            self.calibration_quality = params.pop('calibration_quality')
        
        self.calibration_params = params
        print(f"Calibration loaded from {filename}")
    
    def _set_equal_aspect_3d(self, ax, data):
        """Set equal aspect ratio for 3D plots - ensures spheres look like spheres"""
        # Get the range for each axis
        x_range = data[:, 0].max() - data[:, 0].min()
        y_range = data[:, 1].max() - data[:, 1].min()
        z_range = data[:, 2].max() - data[:, 2].min()
        
        # Find the maximum range
        max_range = max(x_range, y_range, z_range)
        
        # Get centers
        x_center = (data[:, 0].max() + data[:, 0].min()) / 2
        y_center = (data[:, 1].max() + data[:, 1].min()) / 2
        z_center = (data[:, 2].max() + data[:, 2].min()) / 2
        
        # Set equal limits
        ax.set_xlim(x_center - max_range/2, x_center + max_range/2)
        ax.set_ylim(y_center - max_range/2, y_center + max_range/2)
        ax.set_zlim(z_center - max_range/2, z_center + max_range/2)
        
        # Force equal aspect ratio
        ax.set_box_aspect([1,1,1])
    
    def _set_equal_aspect_2d(self, ax, x_data, y_data):
        """Set equal aspect ratio for 2D plots - ensures circles look like circles"""
        ax.set_aspect('equal', adjustable='box')
        
        # Optional: also set equal limits for better visualization
        x_range = x_data.max() - x_data.min()
        y_range = y_data.max() - y_data.min()
        max_range = max(x_range, y_range)
        
        x_center = (x_data.max() + x_data.min()) / 2
        y_center = (y_data.max() + y_data.min()) / 2
        
        margin = max_range * 0.05  # 5% margin
        ax.set_xlim(x_center - max_range/2 - margin, x_center + max_range/2 + margin)
        ax.set_ylim(y_center - max_range/2 - margin, y_center + max_range/2 + margin)
    
    def visualize_calibration(self, output_dir=None, save_plots=True):
        """Enhanced visualization with properly proportioned plots"""
        if len(self.raw_data) == 0 or self.calibration_params is None:
            print("Need both raw data and calibration to visualize")
            return
        
        raw_data = np.array(self.raw_data)
        calibrated_data = np.array([self.apply_calibration(point) for point in self.raw_data])
        
        # Set up dark theme
        plt.style.use('dark_background')
        fig = plt.figure(figsize=(20, 12))
        
        # Raw data 3D plot
        ax1 = fig.add_subplot(231, projection='3d')
        ax1.scatter(raw_data[:, 0], raw_data[:, 1], raw_data[:, 2], 
                   c='red', s=2, alpha=0.7, edgecolors='none')
        ax1.set_title('Raw Magnetometer Data', fontsize=14, fontweight='bold')
        ax1.set_xlabel('X (µT)', fontsize=12)
        ax1.set_ylabel('Y (µT)', fontsize=12)
        ax1.set_zlabel('Z (µT)', fontsize=12)
        
        # Apply equal aspect ratio to raw data
        self._set_equal_aspect_3d(ax1, raw_data)
        
        # Calibrated data 3D plot
        ax2 = fig.add_subplot(232, projection='3d')
        ax2.scatter(calibrated_data[:, 0], calibrated_data[:, 1], calibrated_data[:, 2],
                   c='cyan', s=2, alpha=0.7, edgecolors='none')
        ax2.set_title('Calibrated Magnetometer Data', fontsize=14, fontweight='bold')
        ax2.set_xlabel('X (µT)', fontsize=12)
        ax2.set_ylabel('Y (µT)', fontsize=12)
        ax2.set_zlabel('Z (µT)', fontsize=12)
        
        # Apply equal aspect ratio to calibrated data
        self._set_equal_aspect_3d(ax2, calibrated_data)
        
        # Add reference sphere to calibrated plot
        if self.calibration_quality:
            u = np.linspace(0, 2 * np.pi, 30)
            v = np.linspace(0, np.pi, 30)
            r = self.calibration_quality['mean_radius']
            
            # Center the sphere at origin (since calibrated data should be centered)
            x_sphere = r * np.outer(np.cos(u), np.sin(v))
            y_sphere = r * np.outer(np.sin(u), np.sin(v))
            z_sphere = r * np.outer(np.ones(np.size(u)), np.cos(v))
            ax2.plot_surface(x_sphere, y_sphere, z_sphere, alpha=0.2, color='yellow')
        
        # Magnitude comparison histogram
        ax3 = fig.add_subplot(233)
        raw_magnitudes = np.linalg.norm(raw_data, axis=1)
        cal_magnitudes = np.linalg.norm(calibrated_data, axis=1)
        ax3.hist(raw_magnitudes, alpha=0.6, label='Raw', bins=50, color='red', density=True)
        ax3.hist(cal_magnitudes, alpha=0.6, label='Calibrated', bins=50, color='cyan', density=True)
        ax3.set_title('Magnitude Distribution', fontsize=14, fontweight='bold')
        ax3.set_xlabel('Magnitude (µT)', fontsize=12)
        ax3.set_ylabel('Density', fontsize=12)
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # XY plane projections with equal aspect
        ax4 = fig.add_subplot(234)
        ax4.scatter(raw_data[:, 0], raw_data[:, 1], c='red', s=1, alpha=0.5, label='Raw')
        ax4.scatter(calibrated_data[:, 0], calibrated_data[:, 1], c='cyan', s=1, alpha=0.5, label='Calibrated')
        ax4.set_title('XY Plane Projection', fontsize=14, fontweight='bold')
        ax4.set_xlabel('X (µT)', fontsize=12)
        ax4.set_ylabel('Y (µT)', fontsize=12)
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # Apply equal aspect to XY projection
        all_x = np.concatenate([raw_data[:, 0], calibrated_data[:, 0]])
        all_y = np.concatenate([raw_data[:, 1], calibrated_data[:, 1]])
        self._set_equal_aspect_2d(ax4, all_x, all_y)
        
        # XZ plane projections with equal aspect
        ax5 = fig.add_subplot(235)
        ax5.scatter(raw_data[:, 0], raw_data[:, 2], c='red', s=1, alpha=0.5, label='Raw')
        ax5.scatter(calibrated_data[:, 0], calibrated_data[:, 2], c='cyan', s=1, alpha=0.5, label='Calibrated')
        ax5.set_title('XZ Plane Projection', fontsize=14, fontweight='bold')
        ax5.set_xlabel('X (µT)', fontsize=12)
        ax5.set_ylabel('Z (µT)', fontsize=12)
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # Apply equal aspect to XZ projection
        all_x = np.concatenate([raw_data[:, 0], calibrated_data[:, 0]])
        all_z = np.concatenate([raw_data[:, 2], calibrated_data[:, 2]])
        self._set_equal_aspect_2d(ax5, all_x, all_z)
        
        # Quality metrics text
        ax6 = fig.add_subplot(236)
        ax6.axis('off')
        quality_text = "Calibration Quality Metrics\n\n"
        if self.calibration_quality:
            quality_text += f"Method: {self.calibration_params['method'].title()}\n"
            quality_text += f"Mean Radius: {self.calibration_quality['mean_radius']:.3f} µT\n"
            quality_text += f"Radius Std Dev: {self.calibration_quality['std_radius']:.3f} µT\n"
            quality_text += f"Quality Score: {self.calibration_quality['quality_score']:.4f}\n"
            quality_text += f"Sphericity: {self.calibration_quality['sphericity']:.4f}\n\n"
            
            if self.calibration_quality['quality_score'] < 0.05:
                quality_text += "Status: ✓ Excellent calibration!"
            elif self.calibration_quality['quality_score'] < 0.1:
                quality_text += "Status: ✓ Good calibration"
            elif self.calibration_quality['quality_score'] < 0.2:
                quality_text += "Status: ⚠ Fair calibration"
            else:
                quality_text += "Status: ⚠ Poor calibration"
        
        ax6.text(0.1, 0.9, quality_text, transform=ax6.transAxes, fontsize=12,
                verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='darkgray', alpha=0.8))
        
        plt.tight_layout()
        
        # Save plot if requested
        if save_plots and output_dir:
            os.makedirs(output_dir, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            plot_filename = os.path.join(output_dir, f'magnetometer_calibration_{timestamp}.png')
            plt.savefig(plot_filename, dpi=150, bbox_inches='tight', facecolor='black')
            print(f"Calibration plots saved to: {plot_filename}")
        
        plt.show()
    
    def save_calibration_report(self, output_dir="output"):
        """Save comprehensive calibration report"""
        if self.calibration_params is None:
            print("No calibration to save")
            return
        
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save as C header file for embedded systems
        header_filename = os.path.join(output_dir, f'mag_calibration_{timestamp}.h')
        with open(header_filename, 'w') as f:
            f.write(f"// Magnetometer Calibration Parameters\n")
            f.write(f"// Generated on {timestamp}\n")
            f.write(f"// Method: {self.calibration_params['method']}\n\n")
            f.write(f"#ifndef MAG_CALIBRATION_H\n")
            f.write(f"#define MAG_CALIBRATION_H\n\n")
            
            if self.calibration_params['method'] == 'sphere':
                center = self.calibration_params['center']
                f.write(f"// Hard iron offset (center correction)\n")
                f.write(f"static const float mag_offset[3] = {{\n")
                f.write(f"    {center[0]:.6f}f,  // X offset\n")
                f.write(f"    {center[1]:.6f}f,  // Y offset\n")
                f.write(f"    {center[2]:.6f}f   // Z offset\n")
                f.write(f"}};\n\n")
                
                f.write(f"// Simple calibration function\n")
                f.write(f"static inline void apply_mag_calibration(float raw[3], float cal[3]) {{\n")
                f.write(f"    cal[0] = raw[0] - mag_offset[0];\n")
                f.write(f"    cal[1] = raw[1] - mag_offset[1];\n")
                f.write(f"    cal[2] = raw[2] - mag_offset[2];\n")
                f.write(f"}}\n\n")
                
            elif self.calibration_params['method'] == 'ellipsoid':
                center = self.calibration_params['center']
                transform = self.calibration_params['transform_matrix']
                
                f.write(f"// Hard iron offset\n")
                f.write(f"static const float mag_offset[3] = {{\n")
                f.write(f"    {center[0]:.6f}f,  // X offset\n")
                f.write(f"    {center[1]:.6f}f,  // Y offset\n")
                f.write(f"    {center[2]:.6f}f   // Z offset\n")
                f.write(f"}};\n\n")
                
                f.write(f"// Soft iron correction matrix\n")
                f.write(f"static const float mag_transform[3][3] = {{\n")
                for i in range(3):
                    f.write(f"    {{")
                    for j in range(3):
                        f.write(f"{transform[i,j]:.6f}f")
                        if j < 2:
                            f.write(f", ")
                    f.write(f"}}")
                    if i < 2:
                        f.write(f",")
                    f.write(f"\n")
                f.write(f"}};\n\n")
                
                f.write(f"// Full calibration function\n")
                f.write(f"static inline void apply_mag_calibration(float raw[3], float cal[3]) {{\n")
                f.write(f"    float centered[3] = {{\n")
                f.write(f"        raw[0] - mag_offset[0],\n")
                f.write(f"        raw[1] - mag_offset[1],\n")
                f.write(f"        raw[2] - mag_offset[2]\n")
                f.write(f"    }};\n\n")
                f.write(f"    cal[0] = mag_transform[0][0] * centered[0] + mag_transform[0][1] * centered[1] + mag_transform[0][2] * centered[2];\n")
                f.write(f"    cal[1] = mag_transform[1][0] * centered[0] + mag_transform[1][1] * centered[1] + mag_transform[1][2] * centered[2];\n")
                f.write(f"    cal[2] = mag_transform[2][0] * centered[0] + mag_transform[2][1] * centered[1] + mag_transform[2][2] * centered[2];\n")
                f.write(f"}}\n\n")
            
            f.write(f"#endif // MAG_CALIBRATION_H\n")
        
        print(f"Calibration report saved to: {header_filename}")


# Example usage
if __name__ == "__main__":
    # Initialize calibrator
    cal = MagnetometerCalibrator(port='/dev/tty.usbmodem1101')  # Adjust port as needed
    
    # Option 1: Collect new data
    # if cal.collect_data(duration_minutes=5, min_samples=1000):
    #     cal.save_data_to_file('mag_cal_data.json')
    
    # Option 2: Load existing data
    cal.load_data_from_file('mag_cal_data.json')
    
    # Perform calibration
    cal.calibrate(method='ellipsoid')  # or 'sphere' for simpler calibration
    cal.save_calibration('mag_calibration.json')
    
    # Generate comprehensive report
    cal.save_calibration_report('calibration_output')
    
    # Test calibration on new reading
    # raw_reading = [-47.653, -33.293, -40.227]
    # calibrated = cal.apply_calibration(raw_reading)
    # print(f"Raw: {raw_reading}")
    # print(f"Calibrated: {calibrated}")
    
    # Enhanced visualization with proper aspect ratios
    cal.visualize_calibration(output_dir='calibration_output')