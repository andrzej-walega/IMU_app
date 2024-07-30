
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
#include "../1_IMU_driver/i2c_emul.h"
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
        imu_data_t* data;
        size_t data_lines_number;
        imu_registers_t reg;
    } imu_simul_t;

    bool IMU_sim_set_gyro_freq(uint8_t gyro_freq_reg_val);
    bool IMU_sim_set_gyro_range(uint8_t gyro_range_reg_val);
    bool IMU_sim_set_gyro_mode(uint8_t gyro_mode_reg_val);
    bool IMU_sim_read_gyro_data(uint8_t *gyro_data);
    bool IMU_sim_set_accel_freq(uint8_t accel_freq_reg_val);
    bool IMU_sim_set_accel_range(uint8_t accel_range_reg_val);
    bool IMU_sim_set_accel_mode(uint8_t accel_mode_reg_val);
    bool IMU_sim_set_accel_LP_clk(uint8_t accel_LP_clk_reg_val);
    bool IMU_sim_read_accel_data(uint8_t *accel_data);
    bool IMU_sim_set_idle(uint8_t idle_val);
    bool IMU_sim_read_data(uint8_t start_reg_addr, uint8_t *read_data);
    void IMU_sim_set_data_ready(void);
    void IMU_sim_clear_is_data_ready(void);
    bool load_imu_data_from_csv(const char *filename);
    void get_data_from_arr(uint32_t);
    void convert_float_to_gyro_reg_data(float value, uint8_t *high_byte, uint8_t *low_byte);
    void convert_float_to_accel_reg_data(float value, uint8_t *high_byte, uint8_t *low_byte);
    void update_accel_data();
    void update_gyro_data();
    bool handle_read_register(uint8_t reg_addr, uint8_t *reg_val);
    bool handle_write_register(uint8_t reg_addr, uint8_t *data);
    void process_command();
    bool read_and_answer_command();
    bool send_response();

#ifdef __cplusplus
}
#endif

#endif /* I2C_H_ End of header guard */