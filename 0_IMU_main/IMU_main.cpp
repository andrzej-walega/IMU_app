#include <iostream>
// #include <unistd.h>
// #include <stdint.h>
#include "IMU_main.h"
#include "../1_IMU_driver/ICM_42670_P_driver.h"
#include "../1_IMU_driver/i2c_emul.h"
// #include "../3_IMU_freefall/IMU_freefall.h"

static imu_t imu;

using namespace std;

int main(int argc, char **argv)
{
    // imu.init();
    create_cmd_pipe();
    // uint8_t reg_addr = PWR_MGMT0;
    // uint8_t reg_value = 0xA0;
    // i2c_write_data(reg_addr, &reg_value);
    // i2c_write_data(reg_addr, &reg_value);
    // i2c_write_data(reg_addr, &reg_value);
    // i2c_read_data(reg_addr, &reg_value);
    // i2c_read_data(reg_addr, &reg_value);
    // i2c_read_data(reg_addr, &reg_value);

    IMU_set_accel_range(ACCEL_UI_FS_SEL_2G);
    IMU_set_gyro_range(GYRO_UI_FS_SEL_250DPS);
    IMU_set_accel_freq(ACCEL_ODR_50HZ);
    IMU_set_gyro_freq(GYRO_ODR_50HZ);
    imu.acquisition_started = IMU_start_acquisition();
    cout << "acquisition_started" << imu.acquisition_started << endl;
    unlink_cmd_pipe();
    // Main loop
    while (1)
    {

        return 0;
    }
}
