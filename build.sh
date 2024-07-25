#!/bin/bash

# Navigate to the script directory (assuming it's placed in the root of your project)
cd "$(dirname "$0")"

# Remove any existing build directory
rm -rf build

# Create and navigate to the new build directory
mkdir build
cp imu_data.csv ./build
cd build

# Run cmake and build the project
cmake -G "Unix Makefiles" ..
cmake --build .

