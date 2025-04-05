# Run this script to plot data from the serial port
# Note: you might need to run "pip install pyserial"

import serial
import matplotlib.pyplot as plt
from collections import deque
import argparse



# Parse args --------------------
parser = argparse.ArgumentParser()
# Change this for your device
# On linux, probably something like /dev/ttyACM0
parser.add_argument("--port", type=str, required=False, default="/dev/cu.usbmodem101")

# Number of points to plot
parser.add_argument("--n", type=int, required=True)
parser.add_argument("--max_points", type=int, required=False, default=200)

args = parser.parse_args()
# -----------------------------

max_points = args.max_points
num_values = args.n
ser = serial.Serial('/dev/cu.usbmodem101', 115200, timeout=1)

# Configure matplotlib
plt.ion()

fig, ax = plt.subplots()


data = [deque([], maxlen=max_points) for _ in range(num_values)]
lines = [ax.plot(data[i], label=f"#{i+1}")[0] for i in range(num_values)]

ax.grid(True)
ax.legend()

try:
    while True:
        # Read as much data as possible
        if ser.in_waiting:

            # Strip line
            line_raw = ser.readline().decode('utf-8', errors='replace').strip()
            line_after_info = line_raw.split("[INFO] ")[-1].strip()

            # Extract numbers from message
            try:
                nums = [float(n) for n in line_after_info.split(",")]

                for (i, n) in enumerate(nums):
                    data[i].append(n)
                    lines[i].set_ydata(data[i])
                    lines[i].set_xdata(range(len(data[i])))

            except Exception as e:
                print(f"Rejected invalid line: '{line_raw}' [error = '{e}']")

        ax.relim()
        ax.autoscale_view(True, True, True)
        plt.pause(0.01)
except:
    print("Exiting...")
finally:
    ser.close()

