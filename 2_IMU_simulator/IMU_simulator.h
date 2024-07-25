
#ifndef IMU_SIMULATOR_H_
#define IMU_SIMULATOR_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "../1_IMU_driver/i2c.h"
#include "../1_IMU_driver/ICM_42670_P_driver.h"

    typedef struct
    {
        float ax, ay, az;
        float gx, gy, gz;
    } imu_data_t;

    typedef struct
    {
        uint8_t accel_data_x1;
        uint8_t accel_data_x0;
        uint8_t accel_data_y1;
        uint8_t accel_data_y0;
        uint8_t accel_data_z1;
        uint8_t accel_data_z0;
        uint8_t gyro_data_x1;
        uint8_t gyro_data_x0;
        uint8_t gyro_data_y1;
        uint8_t gyro_data_y0;
        uint8_t gyro_data_z1;
        uint8_t gyro_data_z0;
        uint8_t pwr_mgmt0;
        uint8_t gyro_config0;
        uint8_t accel_config0;
        uint8_t int_status_drdy;
    } imu_registers_t;

    typedef struct
    {
        uint8_t address;
        uint16_t gyro_freq;
        uint16_t gyro_range;
        bool gyro_on;
        uint16_t accel_freq;
        uint8_t accel_range;
        bool accel_on;
        bool data_ready;
        imu_data_t *data;
        size_t data_length;
        size_t current_index;
        imu_registers_t reg;
    } imu_simul_t;

    bool load_imu_data_from_csv(const char *filename);
    void get_accel_data_from_arr(uint32_t);
    void convert_float_to_gyro_reg_data(float value, uint8_t *high_byte, uint8_t *low_byte);
    void convert_float_to_accel_reg_data(float value, uint8_t *high_byte, uint8_t *low_byte);
    void update_accel_data();
    void update_gyro_data();
    bool handle_read_register(uint8_t reg_addr, uint8_t *reg_val);
    bool handle_write_register(uint8_t reg_addr, uint8_t *data);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* I2C_H_ End of header guard */