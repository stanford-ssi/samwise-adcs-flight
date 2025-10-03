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
import threading

class MagnetometerCalibrator:
    def __init__(self, port='/dev/tty.usbmodem101', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.raw_data = []
        self.calibration_params = None
        self.calibration_quality = None
        self.plot_enabled = False
        self.fig = None
        self.ax = None
        self.scatter = None
        
    def setup_realtime_plot(self):
        """Setup real-time 3D plot for data visualization"""
        plt.ion()  # Enable interactive mode
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X (µT)')
        self.ax.set_ylabel('Y (µT)')
        self.ax.set_zlabel('Z (µT)')
        self.ax.set_title('Real-time Magnetometer Data Collection')
        
        # Set initial axis limits (will be updated dynamically)
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(-1, 1)
        
        plt.show(block=False)
        self.plot_enabled = True

    def update_plot(self):
        """Update the 3D plot with current data"""
        if not self.plot_enabled or len(self.raw_data) == 0:
            return
        
        try:
            data = np.array(self.raw_data)
            
            # Clear and replot
            self.ax.clear()
            
            # Color points by order (newer points are brighter)
            colors = np.arange(len(data))
            self.ax.scatter(data[:, 0], data[:, 1], data[:, 2], 
                          c=colors, cmap='viridis', s=20, alpha=0.7)
            
            # Update axis limits based on data
            margin = 0.1
            for i, axis in enumerate(['x', 'y', 'z']):
                data_min, data_max = data[:, i].min(), data[:, i].max()
                range_size = data_max - data_min
                if range_size > 0:
                    getattr(self.ax, f'set_{axis}lim')(
                        data_min - margin * range_size, 
                        data_max + margin * range_size
                    )
            
            self.ax.set_xlabel('X (µT)')
            self.ax.set_ylabel('Y (µT)')
            self.ax.set_zlabel('Z (µT)')
            
            # Calculate and display sphericity when we have enough data
            sphericity_text = ""
            if len(data) > 100:
                center = np.mean(data, axis=0)
                distances = np.linalg.norm(data - center, axis=1)
                mean_distance = np.mean(distances)
                std_distance = np.std(distances)
                quality_score = std_distance / mean_distance
                sphericity = 1.0 - quality_score
                
                sphericity_text = f" | Sphericity: {sphericity:.3f}"
            
            self.ax.set_title(f'Magnetometer Data ({len(data)} points){sphericity_text}')
            
            plt.draw()
            plt.pause(0.001)  # Reduced pause time
            
        except Exception as e:
            print(f"Plot update error: {e}")

    def collect_data(self, min_samples=1000, enable_plot=True):
        """Collect magnetometer data over serial for calibration with auto-reconnect"""
        print(f"Collecting {min_samples} magnetometer samples...")
        print("Rotate the satellite/magnetometer in all orientations!")
        
        if enable_plot:
            self.setup_realtime_plot()
        
        start_time = time.time()
        sample_count = 0
        reconnect_delay = 3 # seconds
        ser = None
        pattern = r'Magnetometer reading: \[([-\d.]+), ([-\d.]+), ([-\d.]+)\]'
        
        while sample_count < min_samples:
            try:
                # Establish connection if needed
                if ser is None or not ser.is_open:
                    print(f"\nConnecting to {self.port}...")
                    ser = serial.Serial(self.port, self.baudrate, timeout=1)
                    print("Connected!")
                
                # Try to read data
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                match = re.search(pattern, line)
                if match:
                    x, y, z = map(float, match.groups())
                    self.raw_data.append([x, y, z])
                    sample_count += 1
                    
                    # Update plot every 25 samples to avoid slowing down data collection
                    if self.plot_enabled and sample_count % 25 == 0:
                        self.update_plot()
                    
                    # Print progress
                    elapsed = time.time() - start_time
                    rate = sample_count / elapsed if elapsed > 0 else 0
                    print(f"\rCollected {sample_count} samples... ({rate:.1f} samples/sec)", end='')
                
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
        
        # Final plot update
        if self.plot_enabled:
            self.update_plot()
            plt.ioff()  # Turn off interactive mode
        
        return len(self.raw_data) >= min_samples
    
    def load_data_from_file(self, filename):
        """Load previously collected data from file"""
        with open(filename, 'r') as f:
            self.raw_data = json.load(f)
        print(f"Loaded {len(self.raw_data)} samples from {filename}")
    
    def save_data_to_file(self, filename=None, output_dir="output"):
        """Save collected data to file"""
        if filename is None:
            os.makedirs(output_dir, exist_ok=True)
            self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(output_dir, f'mag_calibration_{self.timestamp}.json')
        
        with open(filename, 'w') as f:
            json.dump(self.raw_data, f)
        print(f"Saved {len(self.raw_data)} samples to {filename}")
        return filename
    
    def sphere_fit(self, data):
        """Simple sphere calibration: only corrects hard iron effects (offset)"""
        # Find center that minimizes variance in distances from center
        center = np.mean(data, axis=0)
        
        # Iteratively refine center to minimize distance variance
        for _ in range(10):
            distances = np.linalg.norm(data - center, axis=1)
            mean_radius = np.mean(distances)
            
            # Adjust center towards points that are too far/close
            for i, point in enumerate(data):
                error = distances[i] - mean_radius
                direction = (point - center) / distances[i] if distances[i] > 0 else 0
                center += 0.001 * error * direction
        
        # Final radius calculation
        distances = np.linalg.norm(data - center, axis=1)
        radius = np.mean(distances)
        
        return {
            'center': center,
            'radius': radius,
            'transform_matrix': np.eye(3)  # Identity matrix - no scaling/rotation correction
        }
    
    def ellipsoid_fit(self, data):
        """Advanced calibration: fit ellipsoid to correct for both hard iron and soft iron effects"""
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
    
    def calibrate(self, method='ellipsoid'):
        """Perform calibration using specified method"""
        if len(self.raw_data) < 500:
            raise Exception("Need at least 500 samples for calibration")
        
        data = np.array(self.raw_data)
    
        if method == 'sphere':
            self.calibration_params = self.sphere_fit(data)
            self.calibration_params['method'] = 'sphere'
            print(f"Calibration complete using sphere method (hard iron correction only)")
        else:
            self.calibration_params = self.ellipsoid_fit(data)
            self.calibration_params['method'] = 'ellipsoid'
            print(f"Calibration complete using ellipsoid method (hard + soft iron correction)")
        
        # Calculate calibration quality
        self._evaluate_calibration_quality()
        
        return self.calibration_params
    
    def apply_calibration(self, raw_reading):
        """Apply calibration to a raw magnetometer reading"""
        if self.calibration_params is None:
            return raw_reading
        
        # Convert to numpy array
        reading = np.array(raw_reading)
        
        # Apply hard iron correction (subtract center)
        corrected = reading - self.calibration_params['center']
        
        # Apply soft iron correction (transform matrix)
        calibrated = self.calibration_params['transform_matrix'] @ corrected
        
        return calibrated
    
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
            print("Excellent calibration")
        elif quality_score < 0.1:
            print("Good calibration")
        elif quality_score < 0.2:
            print("Fair calibration - consider collecting more data")
        else:
            print("Poor calibration - more data needed or check sensor mounting")
    
    def save_calibration(self, output_dir="output"):
        """Save comprehensive calibration report"""
        if self.calibration_params is None:
            print("No calibration to save")
            return
        
        os.makedirs(output_dir, exist_ok=True)
        timestamp = getattr(self, 'timestamp', datetime.now().strftime("%Y%m%d_%H%M%S"))
        
        # Save as C header file for embedded systems
        header_filename = os.path.join(output_dir, f'mag_calibration_{timestamp}.h')
        with open(header_filename, 'w') as f:
            f.write(f"/**\n")
            f.write(f" * mag_calibration_{timestamp}.h\n")
            f.write(f" * \n")
            f.write(f" * Method: {self.calibration_params['method']}\n")
            f.write(f" * Samples: {len(self.raw_data)}\n")
            if self.calibration_quality:
                f.write(f" * Sphericity: {self.calibration_quality['sphericity']:.4f}\n")
                f.write(f" * Mean Radius: {self.calibration_quality['mean_radius']:.3f} µT\n")
                f.write(f" * Radius Std Dev: {self.calibration_quality['std_radius']:.3f} µT\n")
            f.write(f" */\n\n")
            f.write(f"// Hard iron offset correction (sensor units)\n")
            f.write(f"// Set to zeros during calibration\n")
            f.write(f"constexpr float3 MAG_HARD_IRON_OFFSET = float3{{{self.calibration_params['center'][0]:.6f}, {self.calibration_params['center'][1]:.6f}, {self.calibration_params['center'][2]:.6f}}};\n\n")

            f.write(f"// Soft iron matrix correction (in sensor units)\n")
            f.write(f"// Set to identity during calibration\n")
            f.write(f"constexpr float3x3 MAG_SOFT_IRON_MATRIX = {{\n")
            transform = self.calibration_params['transform_matrix']
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
        
        print(f"Calibration report saved to: {header_filename}")

# Example usage
if __name__ == "__main__":
    print("=" * 60)
    print("MAGNETOMETER CALIBRATION SCRIPT")
    print("=" * 60)
    print("\nIMPORTANT: Before starting calibration:")
    print("1. Go to src/drivers/magnetometer.cpp")
    print("2. Comment out the normalization line in rm3100_get_reading()")
    print("3. Flash the updated firmware to your device")
    print("4. We need raw magnetometer values, not normalized ones!")
    print("\nPress Enter when ready to continue...")
    input()
    
    # Initialize calibrator
    cal = MagnetometerCalibrator(port='/dev/tty.usbmodem101')  # Adjust port as needed
    
    # Collect new data and save it
    if cal.collect_data(min_samples=10000):
        json_filename = cal.save_data_to_file()
        # Load the data we just saved (demonstrates the load functionality)
        cal.load_data_from_file(json_filename)
    
    # Perform calibration (choose method)
    cal.calibrate(method='sphere') 
    
    # Generate comprehensive report
    cal.save_calibration()