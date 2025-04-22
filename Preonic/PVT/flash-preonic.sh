#!/bin/bash

# Flash Preonic UF2 firmware
# This script monitors for a Preonic keyboard in bootloader mode and flashes the firmware

FIRMWARE="Preonic.ino.uf2"
TARGET_DRIVE="/Volumes/PREONICBOOT"

echo "Preonic UF2 Firmware Flasher"
echo "Waiting for Preonic in bootloader mode..."

while true; do
    if [ -d "$TARGET_DRIVE" ]; then
        echo "Found Preonic bootloader drive"
        sleep 1
        if [ -f "$FIRMWARE" ]; then
            echo "Copying firmware..."
            # Redirect stderr to suppress expected unmount errors
            cp "$FIRMWARE" "$TARGET_DRIVE/" 2>&1 | grep -v "fcopyfile failed: Input/output error" | grep -v "fchmod failed: No such file or directory" || true
            
            # Brief pause to let any errors display
            sleep 0.5
            echo "Waiting for next flash request..."
        else
            echo "Error: $FIRMWARE not found in current directory"
            exit 1
        fi
    fi
    sleep 1
done
