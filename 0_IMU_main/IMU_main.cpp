#include <iostream>
#include "IMU_main.h"
#include "../1_IMU_driver/ICM_42670_P_driver.h"
#include "../1_IMU_driver/i2c_emul.h"
// #include "../3_IMU_freefall/IMU_freefall.h"

// static imu_t imu;

using namespace std;

int main(int argc, char **argv)
{
    std::cout << "main 0" << std::endl;
    IMU_init(IMU_ADDRESS, GYRO_ODR_50HZ, GYRO_UI_FS_SEL_250DPS,
        ACCEL_ODR_50HZ, ACCEL_UI_FS_SEL_2G, 0, 0, 0);
    
    // imu.init();
    // Imu.freefall_init();
    // imu.set_accel_range(ACCEL_UI_FS_SEL_2G);
    // imu.set_gyro_range(GYRO_UI_FS_SEL_250DPS);
    // imu.set_accel_freq(ACCEL_ODR_50HZ);
    // imu.set_gyro_freq(GYRO_ODR_50HZ);
    // imu.acquisition_started;
    // imu.read_data();
    // imu.set_freefall_threshold();
    // imu_freefall_detect();


    IMU_set_accel_range(ACCEL_UI_FS_SEL_2G);
    IMU_set_gyro_range(GYRO_UI_FS_SEL_250DPS);
    IMU_set_accel_freq(ACCEL_ODR_50HZ);
    IMU_set_gyro_freq(GYRO_ODR_50HZ);
    IMU_start_acquisition();
    uint8_t accel_data[6] = {0};
    // Main loop
    while (1)
    {
        IMU_read_accel_data(accel_data);
        IMU_read_gyro_data(accel_data);
    }
}
