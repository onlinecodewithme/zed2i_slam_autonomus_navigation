#!/bin/bash

# Script to check for and display costmaps

MAPS_DIR=~/maps
TEMP_DIR=/tmp

echo "=== Costmap Status ==="

# Check if maps directory exists
if [ ! -d "$MAPS_DIR" ]; then
    echo "Error: Maps directory '$MAPS_DIR' does not exist. Creating it now."
    mkdir -p $MAPS_DIR
else
    echo "Maps directory: $MAPS_DIR"
    
    # Count costmap files in maps directory
    COSTMAP_COUNT=$(find $MAPS_DIR -name "autonomous_inspection_costmap_*.pgm" | wc -l)
    if [ $COSTMAP_COUNT -gt 0 ]; then
        echo "Found $COSTMAP_COUNT costmap files in $MAPS_DIR"
        ls -la $MAPS_DIR/autonomous_inspection_costmap_*.pgm 2>/dev/null || true
        
        # Display the latest costmap if image viewer is available
        LATEST_COSTMAP=$(find $MAPS_DIR -name "autonomous_inspection_costmap_*.pgm" | sort | tail -n 1)
        if [ -n "$LATEST_COSTMAP" ]; then
            echo "Latest costmap: $LATEST_COSTMAP"
            if command -v display &> /dev/null; then
                echo "Displaying costmap with ImageMagick..."
                DISPLAY=:1 display $LATEST_COSTMAP &
            elif command -v ristretto &> /dev/null; then
                echo "Displaying costmap with Ristretto..."
                DISPLAY=:1 ristretto $LATEST_COSTMAP &
            else
                echo "No image viewer found. Install ImageMagick or Ristretto to view costmaps."
            fi
        fi
    else
        echo "No costmap files found in $MAPS_DIR"
    fi
fi

# Check if there are any costmaps in /tmp
TEMP_COSTMAP_COUNT=$(find $TEMP_DIR -name "costmap_*.pgm" | wc -l)
if [ $TEMP_COSTMAP_COUNT -gt 0 ]; then
    echo "Found $TEMP_COSTMAP_COUNT temporary costmap files in $TEMP_DIR"
    ls -la $TEMP_DIR/costmap_*.pgm 2>/dev/null || true
    
    # Copy temp costmaps to maps directory
    echo "Copying temporary costmaps to $MAPS_DIR..."
    cp -f $TEMP_DIR/costmap_*.pgm $MAPS_DIR/ 2>/dev/null || true
    echo "Done!"
else
    echo "No temporary costmap files found in $TEMP_DIR"
fi

echo "=== End of Status ==="

# Provide hint if no costmaps found
if [ $COSTMAP_COUNT -eq 0 ] && [ $TEMP_COSTMAP_COUNT -eq 0 ]; then
    echo ""
    echo "No costmaps found. To generate costmaps, run:"
    echo "  ./src/xavier_robotics/scripts/launch_inspection.sh"
    echo ""
    echo "You can also check if the costmap_saver node is running:"
    echo "  ros2 node list | grep costmap_saver"
fi
