import numpy as np
import serial
import re
import time
import json
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class MagnetometerCalibrator:
    def __init__(self, port='/dev/tty.usbmodem1101', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.raw_data = []
        self.calibration_params = None
        
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
        
        return {
            'center': center,
            'transform_matrix': eigenvecs,
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
        return self.calibration_params
    
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
            scales = self.calibration_params['scale_factors']
            
            # Apply transformation
            centered = raw - center
            transformed = transform.T @ centered
            calibrated = transform @ (transformed * scales)
        
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
        
        self.calibration_params = params
        print(f"Calibration loaded from {filename}")
    
    def visualize_calibration(self):
        """Plot raw vs calibrated data for visualization"""
        if len(self.raw_data) == 0 or self.calibration_params is None:
            print("Need both raw data and calibration to visualize")
            return
        
        raw_data = np.array(self.raw_data)
        calibrated_data = np.array([self.apply_calibration(point) for point in self.raw_data])
        
        fig = plt.figure(figsize=(15, 5))
        
        # Raw data
        ax1 = fig.add_subplot(131, projection='3d')
        ax1.scatter(raw_data[:, 0], raw_data[:, 1], raw_data[:, 2], alpha=0.6)
        ax1.set_title('Raw Magnetometer Data')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')
        
        # Calibrated data
        ax2 = fig.add_subplot(132, projection='3d')
        ax2.scatter(calibrated_data[:, 0], calibrated_data[:, 1], calibrated_data[:, 2], alpha=0.6, color='red')
        ax2.set_title('Calibrated Magnetometer Data')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_zlabel('Z')
        
        # Comparison of magnitudes
        ax3 = fig.add_subplot(133)
        raw_magnitudes = np.linalg.norm(raw_data, axis=1)
        cal_magnitudes = np.linalg.norm(calibrated_data, axis=1)
        ax3.hist(raw_magnitudes, alpha=0.5, label='Raw', bins=50)
        ax3.hist(cal_magnitudes, alpha=0.5, label='Calibrated', bins=50)
        ax3.set_title('Magnitude Distribution')
        ax3.set_xlabel('Magnitude')
        ax3.set_ylabel('Count')
        ax3.legend()
        
        plt.tight_layout()
        plt.show()


# Example usage
if __name__ == "__main__":
    # Initialize calibrator
    cal = MagnetometerCalibrator(port='/dev/tty.usbmodem1101')  # Adjust port as needed
    
    # Option 1: Collect new data
    # if cal.collect_data(duration_minutes=2, min_samples=1000):
    #     cal.save_data_to_file('mag_cal_data.json')
    
    # Option 2: Load existing data
    cal.load_data_from_file('mag_cal_data.json')
    
    # Perform calibration
    cal.calibrate(method='ellipsoid')  # or 'sphere' for simpler calibration
    cal.save_calibration('mag_calibration.json')
    
    # Test calibration on new reading
    # raw_reading = [-47.653, -33.293, -40.227]
    # calibrated = cal.apply_calibration(raw_reading)
    # print(f"Raw: {raw_reading}")
    # print(f"Calibrated: {calibrated}")
    
    # Visualize results
    # cal.visualize_calibration()