
//----------------------------------------------------------------
// TDK Invensense ICM-42670-P IMU driver
//----------------------------------------------------------------

#ifndef IMU_DRIVER_H_
#define IMU_DRIVER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include "i2c.h"

// Registers addresses
#define IMU_ADDRESS 0x68

#define ACCEL_DATA_X1 0x0B
#define ACCEL_DATA_X0 0x0C
#define ACCEL_DATA_Y1 0x0D
#define ACCEL_DATA_Y0 0x0E
#define ACCEL_DATA_Z1 0x0F
#define ACCEL_DATA_Z0 0x10
#define GYRO_DATA_X1 0x11
#define GYRO_DATA_X0 0x12
#define GYRO_DATA_Y1 0x13
#define GYRO_DATA_Y0 0x14
#define GYRO_DATA_Z1 0x15
#define GYRO_DATA_Z0 0x16
#define PWR_MGMT0 0x1F
#define GYRO_CONFIG0 0x20
#define ACCEL_CONFIG0 0x21
#define INT_STATUS_DRDY 0x39

// PWR_MGMT0
#define ACCEL_LP_CLK_SEL_POS 7
#define IDLE_POS 4
#define GYRO_MODE_POS 2
#define ACCEL_MODE_POS 0
// GYRO_CONFIG0
#define GYRO_UI_FS_SEL_POS 5
#define GYRO_ODR_POS 0
//ACCEL_CONFIG0
#define ACCEL_UI_FS_SEL_POS 5
#define ACCEL_ODR_POS 0
// INT_STATUS_DRDY
#define DATA_RDY_INT_POS 0

// Masks defined on the bit positions
#define ACCEL_LP_CLK_SEL_MASK (1 << ACCEL_LP_CLK_SEL_POS)
#define IDLE_MASK (1 << IDLE_POS)
#define GYRO_MODE_MASK (0x03 << GYRO_MODE_POS)
#define ACCEL_MODE_MASK (0x03 << ACCEL_MODE_POS)
#define GYRO_UI_FS_SEL_MASK (0x03 << GYRO_UI_FS_SEL_POS)
#define GYRO_ODR_MASK (0x0F << GYRO_ODR_POS)
#define ACCEL_UI_FS_SEL_MASK (0x03 << ACCEL_UI_FS_SEL_POS)
#define ACCEL_ODR_MASK (0x0F << ACCEL_ODR_POS)
#define DATA_RDY_INT_MASK 1

#define GYRO_UI_FS_SEL_250DPS 0b11
#define GYRO_UI_FS_SEL_500DPS 0b10
#define GYRO_UI_FS_SEL_1000DPS 0b01
#define GYRO_UI_FS_SEL_2000DPS 0b00

#define GYRO_ODR_1600HZ 0b0101
#define GYRO_ODR_800HZ 0b0110
#define GYRO_ODR_400HZ 0b0111
#define GYRO_ODR_200HZ 0b1000
#define GYRO_ODR_100HZ 0b1001
#define GYRO_ODR_50HZ 0b1010
#define GYRO_ODR_25HZ 0b1011
#define GYRO_ODR_12_5HZ 0b1100

#define ACCEL_UI_FS_SEL_2G 0b11
#define ACCEL_UI_FS_SEL_4G 0b10
#define ACCEL_UI_FS_SEL_8G 0b01
#define ACCEL_UI_FS_SEL_16G 0b00

#define ACCEL_ODR_1600HZ 0b0101
#define ACCEL_ODR_800HZ 0b0110
#define ACCEL_ODR_400HZ 0b0111
#define ACCEL_ODR_200HZ 0b1000
#define ACCEL_ODR_100HZ 0b1001
#define ACCEL_ODR_50HZ 0b1010
#define ACCEL_ODR_25HZ 0b1011
#define ACCEL_ODR_12_5HZ 0b1100
#define ACCEL_ODR_6_25HZ 0b1101
#define ACCEL_ODR_3_125HZ 0b1110
#define ACCEL_ODR_1_5625HZ 0b1111

#define ACCEL_LP_CLK_WKUP 0
#define ACCEL_LP_CLK_RC 1
#define IDLE_POS_ON 1
#define IDLE_POS_OFF 0
#define GYRO_MODE_OFF 0b00
#define GYRO_MODE_STANDBY 0b01
#define GYRO_MODE_LOW_NOISE 0b11
#define ACCEL_MODE_OFF 0b00
#define ACCEL_MODE_LOW_POWER 0b10
#define ACCEL_MODE_LOW_NOISE 0b11

#define MAX_I2C_WAIT_TIME_MSEC 1000UL
#define MAX_GYRO_MODE_WAIT_TIME_MSEC 45UL
#define MAX_ACCEL_MODE_WAIT_TIME_MSEC 1UL

#define I2C_BUF_SIZE 255
#define ACCEL_DATA_SIZE 6 /* 16-bit x y z */
#define GYRO_DATA_SIZE 6

    typedef struct
    {
        uint8_t address;
        uint16_t gyro_freq;
        uint16_t gyro_range;
        uint8_t gyro_mode;
        uint16_t accel_freq;
        uint8_t accel_range;
        uint8_t accel_mode;
        uint8_t accel_LP_clk;
        uint8_t idle;
        bool acquisition_started;
        bool data_ready;
    } imu_t;

    bool IMU_init(uint8_t address, uint8_t gyro_freq, uint8_t gyro_range, uint8_t accel_freq, uint8_t accel_range,
        uint8_t i2c_sdl, uint8_t i2c_sdc, uint8_t i2c_hz);
    bool IMU_start_acquisition(void);
    bool IMU_stop_acquisition(void);
    bool IMU_set_gyro_freq(uint16_t gyro_freq);
    bool IMU_set_gyro_range(uint16_t gyro_range);
    bool IMU_set_gyro_mode(uint8_t gyro_mode_reg_val);
    bool IMU_read_gyro_data(uint8_t *gyro_data);
    bool IMU_set_accel_freq(uint16_t accel_freq);
    bool IMU_set_accel_range(uint8_t accel_range);
    bool IMU_set_accel_mode(uint8_t accel_mode_reg_val);
    bool IMU_read_accel_data(uint8_t *accel_data);
    bool IMU_read_data(uint8_t start_reg_addr, uint8_t* read_data, size_t length);
    bool IMU_is_data_ready(void);
    bool IMU_set_accel_LP_clk(uint8_t accel_LP_clk_reg_val);
    bool IMU_set_idle(uint8_t idle_val);
    bool IMU_is_idle(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_DRIVER_H_ End of header guard */
