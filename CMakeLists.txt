
cmake_minimum_required(VERSION 3.10)

# Define the project name
project(IMU_Analyser)

# Specify the C standard
#set(CMAKE_C_STANDARD 99)

#set(CMAKE_BUILD_TYPE Debug)

# Set the output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Include directories
include_directories(0_IMU_main)
include_directories(1_IMU_driver)
include_directories(2_IMU_simulator)
include_directories(3_IMU_freefall)

# Define the source files for imu_main
set(IMU_MAIN_SOURCES
    0_IMU_main/IMU_main.cpp
    0_IMU_main/IMU_main.h
    0_IMU_main/observer.h
    1_IMU_driver/i2c_emul.c
    1_IMU_driver/i2c_emul.h
    1_IMU_driver/i2c.h
    1_IMU_driver/ICM_42670_P_driver.c
    1_IMU_driver/ICM_42670_P_driver.h
    3_IMU_freefall/IMU_freefall.cpp
    3_IMU_freefall/IMU_freefall.h
)

# Define the source files for imu_simulator
set(IMU_SIMULATOR_SOURCES
    2_IMU_simulator/IMU_simulator.c
    2_IMU_simulator/IMU_simulator.h
    1_IMU_driver/i2c_emul.c
    1_IMU_driver/i2c_emul.h
)

# Create the imu_main executable
add_executable(imu_main ${IMU_MAIN_SOURCES})

# Create the imu_simulator executable
add_executable(imu_simulator ${IMU_SIMULATOR_SOURCES})
