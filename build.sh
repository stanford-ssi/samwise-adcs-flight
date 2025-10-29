#!/bin/bash

set -e

# Clean build option
if [[ "$1" == "clean" ]]; then
    echo "Cleaning build directory..."
    rm -rf build
fi

# Create build directory if it doesn't exist
mkdir -p build
cd build

# Configure and build
echo "Configuring project..."
cmake ..

echo "Building project..."
make -j8

# Run tests
echo "Running tests..."
ctest --output-on-failure

# Flash to device
cd ..

# Check if Pico is mounted as USB drive first (BOOTSEL mode)
if [ -d "/Volumes/RP2350" ]; then
    echo "Found RP2350 volume, copying firmware..."
    cp build/samwise-adcs.uf2 /Volumes/RP2350/
    echo "✓ Firmware copied! Device will reboot automatically."
# Otherwise try picotool (check if any RP device is mentioned in output)
elif picotool info 2>&1 | grep -q "RP2350 device"; then
    echo "Found Pico device, flashing with picotool..."
    picotool load build/samwise-adcs.uf2 -f
    echo "✓ Firmware flashed!"
else
    echo "Error: No Pico detected"
    echo "Please connect your RP2350 device"
    exit 1
fi

echo "✓ Build and flash complete!"