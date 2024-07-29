cmake_minimum_required(VERSION 3.10)

# Set the project name
project(IMU_Simulator)

# Specify the C standard
set(CMAKE_C_STANDARD 99)

set(CMAKE_BUILD_TYPE Debug)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/0_IMU_main
    ${CMAKE_SOURCE_DIR}/1_IMU_driver
    ${CMAKE_SOURCE_DIR}/2_IMU_simulator
    ${CMAKE_SOURCE_DIR}/3_IMU_freefall
)

# Add library from the driver source files
add_library(driver STATIC
    ${CMAKE_SOURCE_DIR}/1_IMU_driver/ICM_42670_P_driver.c
    ${CMAKE_SOURCE_DIR}/1_IMU_driver/i2c_emul.c
)

# Add the executables
add_executable(imu_main ${CMAKE_SOURCE_DIR}/0_IMU_main/IMU_main.cpp)
add_executable(imu_simulator ${CMAKE_SOURCE_DIR}/2_IMU_simulator/IMU_simulator.c)

# Link the driver library to the executables
target_link_libraries(imu_main driver)
target_link_libraries(imu_simulator driver)
