#!/bin/bash

# Get the firmware to build from the first argument
FIRMWARE=$1
# Check if the firmware argument is provided
if [ -z "$FIRMWARE" ]; then
    echo "Usage: $0 <firmware>"
    exit 1
fi

make $FIRMWARE
if [ $? -ne 0 ]; then
    echo "Failed to build $FIRMWARE"
    exit 1
fi

# Check if the firmware was built successfully
if [ ! -f "build/$FIRMWARE/$FIRMWARE.bin" ]; then
    echo "Firmware $FIRMWARE not found in build directory"
    exit 1
fi

# Flash the firmware to the device
echo "Flashing $FIRMWARE to the device..."

"/Applications/VESC Tool.app/Contents/MacOS/VESC Tool" --uploadFirmware "build/$FIRMWARE/$FIRMWARE.bin"

echo "Firmware $FIRMWARE flashed successfully!"