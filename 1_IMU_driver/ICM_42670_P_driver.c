
#include <time.h>
#include <stdio.h>
#include <string.h>
#include "ICM_42670_P_driver.h"

static bool IMU_send_I2C_reg_setting(uint8_t reg_addr, uint8_t reg_mask, uint8_t reg_pos, uint8_t set_val);
static imu_t imu;
static clock_t start_time;

static void start_timer()
{
    start_time = clock();
}

static bool time_elapsed(clock_t timeout_msec)
{
    clock_t current_time = clock();
    clock_t elapsed_time = (current_time - start_time);
    return elapsed_time > timeout_msec;
}

// Function implementations

//retval bool: communication ok, returned data is valid
bool IMU_init(uint8_t address, uint8_t gyro_freq, uint8_t gyro_range, uint8_t accel_freq, uint8_t accel_range,
    uint8_t i2c_sdl, uint8_t i2c_sdc, uint8_t i2c_hz)
{
    bool is_init_ok = true;
    imu.address = address;
    imu.gyro_freq = gyro_freq;
    imu.gyro_range = gyro_range;
    imu.gyro_mode = 0;
    imu.accel_freq = accel_freq;
    imu.accel_range = accel_range;
    imu.accel_mode = 0;
    imu.accel_LP_clk = 0;
    imu.idle = 0;
    imu.acquisition_started = false;
    imu.data_ready = false;
    is_init_ok &= i2c_init(i2c_sdl, i2c_sdc, i2c_hz);
    is_init_ok &= IMU_set_gyro_freq(gyro_freq);
    is_init_ok &= IMU_set_gyro_range(gyro_range);
    is_init_ok &= IMU_set_accel_freq(accel_freq);
    is_init_ok &= IMU_set_accel_range(accel_range);
    return is_init_ok;
}

bool IMU_start_acquisition(void)
{
    if (!IMU_set_gyro_mode(GYRO_MODE_LOW_NOISE))
    {
        return false;
    }
    if (!IMU_set_accel_mode(ACCEL_MODE_LOW_NOISE))
    {
        return false;
    }
    imu.acquisition_started = true;
    return true;
}

bool IMU_stop_acquisition(void)
{
    if (!IMU_set_gyro_mode(GYRO_MODE_OFF))
    {
        return false;
    }
    if (!IMU_set_accel_mode(ACCEL_MODE_OFF))
    {
        return false;
    }
    imu.acquisition_started = false;
    return true;
}

bool IMU_set_gyro_freq(uint16_t gyro_freq)
{
    uint16_t gyro_freq_reg_val = 0;
    bool status = false;

    switch (gyro_freq)
    {
    case 1600:
        gyro_freq_reg_val = GYRO_ODR_1600HZ;
        break;
    case 800:
        gyro_freq_reg_val = GYRO_ODR_800HZ;
        break;
    case 400:
        gyro_freq_reg_val = GYRO_ODR_400HZ;
        break;
    case 200:
        gyro_freq_reg_val = GYRO_ODR_200HZ;
        break;
    case 100:
        gyro_freq_reg_val = GYRO_ODR_100HZ;
        break;
    case 50:
        gyro_freq_reg_val = GYRO_ODR_50HZ;
        break;
    case 25:
        gyro_freq_reg_val = GYRO_ODR_25HZ;
        break;
    case 12:
        gyro_freq_reg_val = GYRO_ODR_12_5HZ;
        break;
    default:
        return false;
    }

    if (IMU_send_I2C_reg_setting(GYRO_CONFIG0, GYRO_ODR_MASK, GYRO_ODR_POS, gyro_freq_reg_val))
    {
        imu.gyro_freq = gyro_freq;
        status = true;
    }
    return status;
}

bool IMU_set_accel_freq(uint16_t accel_freq)
{
    uint16_t accel_freq_reg_val = 0;
    bool status = false;

    switch (accel_freq)
    {
    case 1600:
        accel_freq_reg_val = ACCEL_ODR_1600HZ;
        break;
    case 800:
        accel_freq_reg_val = ACCEL_ODR_800HZ;
        break;
    case 400:
        accel_freq_reg_val = ACCEL_ODR_400HZ;
        break;
    case 200:
        accel_freq_reg_val = ACCEL_ODR_200HZ;
        break;
    case 100:
        accel_freq_reg_val = ACCEL_ODR_100HZ;
        break;
    case 50:
        accel_freq_reg_val = ACCEL_ODR_50HZ;
        break;
    case 25:
        accel_freq_reg_val = ACCEL_ODR_25HZ;
        break;
    case 12:
        accel_freq_reg_val = ACCEL_ODR_12_5HZ;
        break;
    case 6:
        accel_freq_reg_val = ACCEL_ODR_6_25HZ;
        break;
    case 3:
        accel_freq_reg_val = ACCEL_ODR_3_125HZ;
        break;
    case 2:
        accel_freq_reg_val = ACCEL_ODR_1_5625HZ;
        break;
    default:
        return false;
    }

    if (IMU_send_I2C_reg_setting(ACCEL_CONFIG0, ACCEL_ODR_MASK, ACCEL_ODR_POS, accel_freq_reg_val))
    {
        imu.accel_freq = accel_freq;
        status = true;
    }
    return status;
}

bool IMU_set_gyro_range(uint16_t gyro_range)
{
    uint16_t gyro_range_reg_val = 0;
    bool status = false;

    switch (gyro_range)
    {
    case 2000:
        gyro_range_reg_val = GYRO_UI_FS_SEL_2000DPS;
        break;
    case 1000:
        gyro_range_reg_val = GYRO_UI_FS_SEL_1000DPS;
        break;
    case 500:
        gyro_range_reg_val = GYRO_UI_FS_SEL_500DPS;
        break;
    case 250:
        gyro_range_reg_val = GYRO_UI_FS_SEL_250DPS;
        break;
    default:
        return false;
    }
    return true;


    if (IMU_send_I2C_reg_setting(GYRO_CONFIG0, GYRO_UI_FS_SEL_MASK, GYRO_UI_FS_SEL_POS, gyro_range_reg_val))
    {
        imu.gyro_range = gyro_range;
        status = true;
    }
    return status;
}

bool IMU_set_accel_range(uint8_t accel_range)
{
    uint8_t accel_range_reg_val = 0;
    bool status = false;

    switch (accel_range)
    {
    case 2:
        accel_range_reg_val = ACCEL_UI_FS_SEL_2G;
        break;
    case 4:
        accel_range_reg_val = ACCEL_UI_FS_SEL_4G;
        break;
    case 8:
        accel_range_reg_val = ACCEL_UI_FS_SEL_8G;
        break;
    case 16:
        accel_range_reg_val = ACCEL_UI_FS_SEL_16G;
        break;
    default:
        return false;
    }
    return true;

    if (IMU_send_I2C_reg_setting(ACCEL_CONFIG0, ACCEL_UI_FS_SEL_MASK, ACCEL_UI_FS_SEL_POS, accel_range_reg_val))
    {
        imu.accel_range = accel_range;
        status = true;
    }
    return status;
}

bool IMU_set_gyro_mode(uint8_t gyro_mode_reg_val)
{
    bool status = false;
    if (IMU_send_I2C_reg_setting(PWR_MGMT0, GYRO_MODE_MASK, GYRO_MODE_POS, gyro_mode_reg_val))
    {
        start_timer();
        while (!time_elapsed(MAX_GYRO_MODE_WAIT_TIME_MSEC))
        {
        }
        imu.gyro_mode = gyro_mode_reg_val;
        status = true;
    }
    return status;
}

bool IMU_set_accel_mode(uint8_t accel_mode_reg_val)
{
    bool status = false;
    if (IMU_send_I2C_reg_setting(PWR_MGMT0, ACCEL_MODE_MASK, ACCEL_MODE_POS, accel_mode_reg_val))
    {
        start_timer();
        while (!time_elapsed(MAX_ACCEL_MODE_WAIT_TIME_MSEC))
        {
        }
        imu.accel_mode = accel_mode_reg_val;
        status = true;
    }
    return status;
}

bool IMU_set_accel_LP_clk(uint8_t accel_LP_clk_reg_val)
{
    bool status = false;
    if (IMU_send_I2C_reg_setting(PWR_MGMT0, ACCEL_LP_CLK_SEL_MASK, ACCEL_LP_CLK_SEL_POS, accel_LP_clk_reg_val))
    {
        imu.accel_LP_clk = accel_LP_clk_reg_val;
        status = true;
    }
    return status;
}

bool IMU_read_gyro_data(uint8_t *gyro_data)
{
    return IMU_read_data(GYRO_DATA_X1, gyro_data, GYRO_DATA_SIZE);
}

bool IMU_read_accel_data(uint8_t *accel_data)
{
    return IMU_read_data(ACCEL_DATA_X1, accel_data, ACCEL_DATA_SIZE);
}

bool IMU_read_data(uint8_t start_reg_addr, uint8_t* read_data, size_t length)
{
    uint8_t reg_address = start_reg_addr;

    if (!imu.acquisition_started)
    {
        return false;
    }

    while (length != 0)
    {
        start_timer();
        while (!IMU_is_data_ready()) // wait for data (INT_STATUS_DRDY)
        {
            if (time_elapsed(MAX_I2C_WAIT_TIME_MSEC))
            {
                printf("IMU: Communication broken, IMU need to be reset!\n");
                return false;
            }
        }

        if (!i2c_read_data(reg_address, read_data))
        {
            return false;
        }
        // printf("read_data reg_address, read_data, reading_bytes %d %d %d\n", reg_address, *read_data, reading_bytes);
        reg_address++;
        read_data++;
        length--;
    }
    return true;
}

bool IMU_set_idle(uint8_t idle_val)
{
    bool status = false;
    if (IMU_send_I2C_reg_setting(PWR_MGMT0, IDLE_MASK, IDLE_POS, idle_val))
    {
        imu.idle = idle_val;
        status = true;
    }
    return status;
}

bool IMU_is_idle(void)
{
    return imu.idle;
}

bool IMU_is_data_ready(void)
{
    uint8_t status;

    if (!i2c_read_data(INT_STATUS_DRDY, &status))
    {
        return false;
    }
    imu.data_ready = (bool)status;
    return imu.data_ready;
}

//retval bool: IMU communication is ok, returned data is valid
static bool IMU_send_I2C_reg_setting(uint8_t reg_addr, uint8_t reg_mask, uint8_t reg_pos, uint8_t set_val)
{

    uint8_t reg_value;

    if (!i2c_read_data(reg_addr, &reg_value))
    {
        return false;
    }

    // Clear the old set_val bits in the reg_value
    reg_value &= ~reg_mask;
    // Set the set_val bits
    reg_value |= (set_val << reg_pos) & reg_mask;

    if (!i2c_write_data(reg_addr, &reg_value))
    {
        return false;
    }
    return true;
}