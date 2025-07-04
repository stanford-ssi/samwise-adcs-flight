#!/usr/bin/env python3
"""
General-purpose serial data logger to CSV
Reads comma-separated values from serial port and saves to CSV file
"""

import serial
import csv
import argparse
import time
from datetime import datetime
import sys
import signal

class SerialCSVLogger:
    def __init__(self, port, baudrate, output_file, headers=None):
        self.port = port
        self.baudrate = baudrate
        self.output_file = output_file
        self.headers = headers
        self.ser = None
        self.csv_file = None
        self.csv_writer = None
        self.running = True
        
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        print("\nInterrupted! Closing connections...")
        self.running = False
        
    def start(self):
        """Start logging serial data to CSV"""
        # Setup signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        
        try:
            # Open serial connection
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to {self.port} at {self.baudrate} baud")
            
            # Open CSV file
            self.csv_file = open(self.output_file, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Write headers if provided
            if self.headers:
                self.csv_writer.writerow(self.headers)
                
            print(f"Logging to {self.output_file}...")
            print("Press Ctrl+C to stop")
            
            row_count = 0
            
            while self.running:
                if self.ser.in_waiting:
                    try:
                        # Read line from serial
                        line_raw = self.ser.readline().decode('utf-8', errors='replace').strip()
                        
                        # Extract data after [INFO] tag if present
                        if "[INFO]" in line_raw:
                            line_data = line_raw.split("[INFO]")[-1].strip()
                        else:
                            line_data = line_raw
                        
                        # Parse comma-separated values
                        values = [v.strip() for v in line_data.split(",")]
                        
                        # Write to CSV
                        self.csv_writer.writerow(values)
                        self.csv_file.flush()  # Ensure data is written
                        
                        row_count += 1
                        
                        # Print progress
                        if row_count % 100 == 0:
                            print(f"Logged {row_count} rows...")
                            
                        # Optional: print the data
                        if row_count <= 5:  # Show first few rows
                            print(f"Row {row_count}: {values}")
                            
                    except Exception as e:
                        print(f"Error parsing line: '{line_raw}' - {e}")
                        
        except serial.SerialException as e:
            print(f"Serial port error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")
        finally:
            self.cleanup()
            
    def cleanup(self):
        """Clean up resources"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial port closed")
            
        if self.csv_file:
            self.csv_file.close()
            print(f"CSV file saved: {self.output_file}")

def main():
    parser = argparse.ArgumentParser(description="Log serial data to CSV file")
    
    # Serial port settings
    parser.add_argument("--port", type=str, required=False, 
                      default="/dev/cu.usbmodem101",
                      help="Serial port (default: /dev/cu.usbmodem101)")
    parser.add_argument("--baudrate", type=int, required=False, 
                      default=115200,
                      help="Baud rate (default: 115200)")
    
    # Output settings
    parser.add_argument("--output", type=str, required=False,
                      default=f"serial_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                      help="Output CSV filename")
    parser.add_argument("--headers", type=str, required=False,
                      help="Comma-separated header names (e.g., 'time,x,y,z')")
    
    args = parser.parse_args()
    
    # Parse headers if provided
    headers = None
    if args.headers:
        headers = [h.strip() for h in args.headers.split(",")]
    
    # Create and start logger
    logger = SerialCSVLogger(
        port=args.port,
        baudrate=args.baudrate,
        output_file=args.output,
        headers=headers
    )
    
    logger.start()

if __name__ == "__main__":
    main()