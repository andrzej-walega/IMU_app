cmake_minimum_required(VERSION 3.10)

# Set the project name
project(IMU_Simulator)

# Specify the C standard
set(CMAKE_C_STANDARD 99)

# Include directories
include_directories(0_IMU_main 1_IMU_driver 2_IMU_simulator 3_IMU_freefall)

# Add library from the driver source files
add_library(ICM_42670_P_driver 1_IMU_driver/ICM_42670_P_driver.c)
add_library(i2c_emul 1_IMU_driver/i2c_emul.c)

# Add the executable
add_executable(imu_main 0_IMU_main/IMU_main.cpp)
add_executable(imu_simulator 2_IMU_simulator/IMU_simulator.c)

# Link the driver library to the executable
target_link_libraries(imu_main ICM_42670_P_driver)
target_link_libraries(imu_main i2c_emul)
target_link_libraries(imu_simulator ICM_42670_P_driver)
