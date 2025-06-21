import serial
import csv
import time
from datetime import datetime

def read_serial_to_csv(port, baudrate=9600, timeout=1, output_file=None):
    """
    Read serial data and save to CSV file
    
    Args:
        port: Serial port (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
        baudrate: Baud rate (default 9600)
        timeout: Read timeout in seconds
        output_file: Output CSV filename (auto-generated if None)
    """

    line_count = 0
    error_count = 0
    
    if output_file is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = f"serial_data_{timestamp}.csv"
    
    try:
        # Open serial connection
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"Connected to {port} at {baudrate} baud")
        
        # Wait for connection to stabilize
        time.sleep(0.1)
        
        # Open CSV file for writing
        with open(output_file, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write header with your specific columns
            writer.writerow(['timestamp', 'Altitude', 'Latitude', 'Longitude', 'B_r', 'B_phi', 'B_theta', 'B_magnitude'])
            
            print("Reading serial data... Press Ctrl+C to stop")
            
            while True:
                try:
                    # Read line from serial
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line:  # Only process non-empty lines
                        timestamp = datetime.now().isoformat()
                        
                        # Parse the data - try different separators
                        data_parts = None
                        
                        # Try comma-separated first
                        data_parts = [part.strip() for part in line.split(',')]

                        # Check if we have the expected number of fields
                        if data_parts and len(data_parts) == 7:
                            # Write parsed data
                            writer.writerow([timestamp] + data_parts)
                            line_count += 1
                        else:
                            # Log problematic lines for debugging
                            error_count += 1
                            if error_count <= 10:  # Only show first 10 errors
                                print(f"Warning: Line {line_count + error_count} has {len(data_parts) if data_parts else 0} fields instead of 7: '{line[:100]}...'")
                            # Optionally save raw line anyway
                            writer.writerow([timestamp, line, '', '', '', '', ''])
                        
                        # Progress indicator
                        total_processed = line_count + error_count
                        
                        # Print progress every 1000 lines
                        if total_processed % 1000 == 0:
                            print(f"Processed {line_count} valid lines, {error_count} errors...")
                        
                        # Stop after expected number of lines (adjust as needed)
                        if total_processed >= 64000:
                            print(f"Reached {total_processed} total lines, stopping...")
                            break
                            
                except UnicodeDecodeError:
                    print("Skipping line due to encoding error")
                    continue
                    
    except KeyboardInterrupt:
        print(f"\nStopped by user. Processed {line_count} valid lines, {error_count} error lines.")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed")
    
    print(f"Data saved to {output_file}")
    print(f"Summary: {line_count} valid data rows, {error_count} problematic lines")
    return output_file

# Example usage
if __name__ == "__main__":
    # Configure these parameters for your setup
    SERIAL_PORT = "/dev/tty.usbmodem1101"  # Change to your port (e.g., "/dev/ttyUSB0" on Linux)
    BAUD_RATE = 115200      # Change to match your device
    
    # Read serial data to CSV
    read_serial_to_csv(SERIAL_PORT, BAUD_RATE)
