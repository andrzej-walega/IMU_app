
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include "IMU_simulator.h"

static imu_simul_t imu;

int main(int argc, char const *argv[])
{
    if (!load_imu_data_from_csv("imu_data.csv"))
    {
        fprintf(stderr, "Failed to load IMU data from CSV file\n");
        return 1;
    }
    create_resp_pipe();
    while(1){
        update_accel_data();
        update_gyro_data();
        read_and_answer_command();
    }
    close_resp_pipe();
    return 0;
}

bool load_imu_data_from_csv(const char *filename)
{
    FILE *file = fopen(filename, "r");
    if (!file)
    {
        perror("Failed to open file");
        return false;
    }

    // Count the number of lines in the file
    char line[256];
    size_t line_count = 0;
    while (fgets(line, sizeof(line), file))
    {
        line_count++;
    }
    rewind(file); // Reset file pointer to the beginning

    // Allocate memory for the IMU data
    imu.data = (imu_data_t *)malloc(sizeof(imu_data_t) * line_count);
    if (imu.data == NULL)
    {
        perror("Failed to allocate memory");
        fclose(file);
        return false;
    }
    imu.data_length = line_count;
    imu.current_index = 0;

    // Read the data into the imu.data
    fgets(line, sizeof(line), file); // Skip the header line
    size_t index = 0;
    while (fgets(line, sizeof(line), file))
    {
        sscanf(line, "%f,%f,%f,%f,%f,%f",
               &imu.data[index].ax, &imu.data[index].ay, &imu.data[index].az,
               &imu.data[index].gx, &imu.data[index].gy, &imu.data[index].gz);
        index++;
    }
    index--;
    // printf("%f, %f, %f, %f, %f, %f \n",
    //        imu.data[index].ax, imu.data[index].ay, imu.data[index].az,
    //        imu.data[index].gx, imu.data[index].gy, imu.data[index].gz);

    fclose(file);
    free(imu.data);
    return true;
}

void get_accel_data_from_arr(uint32_t line_index)
{
    uint8_t axis_number = 3; // x, y, z
    uint8_t *reg_data = &imu.reg.accel_data_x0;
    float *arr_data = &imu.data[line_index].ax;

    for (uint8_t i = 0; i < axis_number; i++)
    {
        convert_float_to_accel_reg_data(*arr_data, reg_data+1, reg_data);
        reg_data + 2;
        arr_data++;
    }
}

void convert_float_to_accel_reg_data(float value, uint8_t *high_byte, uint8_t *low_byte)
{
    int16_t scaled_value;
    float scaling_factor;

    switch (imu.accel_range)
    {
    case 2:
        scaling_factor = 8192.0; // 2^13 / 2G
        break;
    case 4:
        scaling_factor = 4096.0;
        break;
    case 8:
        scaling_factor = 2048.0;
        break;
    case 16:
        scaling_factor = 1024.0;
        break;
    default:
        *high_byte = 0x80;
        *low_byte = 0x00;
        return;
    }

    scaled_value = (int16_t)(value * scaling_factor);
    *high_byte = (scaled_value >> 8) & 0xFF;
    *low_byte = scaled_value & 0xFF;
}

void convert_float_to_gyro_reg_data(float value, uint8_t *high_byte, uint8_t *low_byte)
{
    int16_t scaled_value;
    float scaling_factor;

    switch (imu.gyro_range)
    {
    case 250:
        scaling_factor = 131.0; // 2^15 / 250DPS
        break;
    case 500:
        scaling_factor = 65.5;
        break;
    case 1000:
        scaling_factor = 32.8;
        break;
    case 2000:
        scaling_factor = 16.4;
        break;
    default:
        *high_byte = 0x80;
        *low_byte = 0x00;
        return;
    }

    scaled_value = (int16_t)(value * scaling_factor);
    *high_byte = (scaled_value >> 8) & 0xFF;
    *low_byte = scaled_value & 0xFF;
}

void update_accel_data(void)
{
    static uint32_t accel_line_index = 0;
    static clock_t start_accel_time = 0;

    if (imu.accel_on)
    {
        clock_t current_time = clock();
        if (current_time - start_accel_time >= 1000/imu.accel_freq) //[ms/Hz]
        {
            start_accel_time = current_time;
            get_accel_data_from_arr(accel_line_index);
            IMU_sim_set_data_ready();
        }
    }
    if (accel_line_index >= imu.data_length - 1)
    {
        accel_line_index = 0;
    }
    else
    {
        accel_line_index++;
    }
}

void update_gyro_data(){

}

bool handle_read_register(uint8_t reg_addr, uint8_t *data)
{
    bool result = true;
    switch (reg_addr)
    {
    case ACCEL_DATA_X1:
        *data = imu.reg.accel_data_x1;
        break;
    case ACCEL_DATA_X0:
        *data = imu.reg.accel_data_x0;
        break;
    case ACCEL_DATA_Y1:
        *data = imu.reg.accel_data_y1;
        break;
    case ACCEL_DATA_Y0:
        *data = imu.reg.accel_data_y0;
        break;
    case ACCEL_DATA_Z1:
        *data = imu.reg.accel_data_z1;
        break;
    case ACCEL_DATA_Z0:
        *data = imu.reg.accel_data_z0;
        break;
    case GYRO_DATA_X1:
        *data = imu.reg.gyro_data_x1;
        break;
    case GYRO_DATA_X0:
        *data = imu.reg.gyro_data_x0;
        break;
    case GYRO_DATA_Y1:
        *data = imu.reg.gyro_data_y1;
        break;
    case GYRO_DATA_Y0:
        *data = imu.reg.gyro_data_y0;
        break;
    case GYRO_DATA_Z1:
        *data = imu.reg.gyro_data_z1;
        break;
    case GYRO_DATA_Z0:
        *data = imu.reg.gyro_data_z0;
        break;
    case PWR_MGMT0:
        *data = imu.reg.pwr_mgmt0;
        break;
    case GYRO_CONFIG0:
        *data = imu.reg.gyro_config0;
        break;
    case ACCEL_CONFIG0:
        *data = imu.reg.accel_config0;
        break;
    case INT_STATUS_DRDY:
        *data = imu.reg.int_status_drdy;
        IMU_sim_clear_is_data_ready(); // RC reg => clear after reading
        break;
    default:
        result = false;
        break;
    }
    return result;
}

bool handle_write_register(uint8_t reg_addr, uint8_t *data)
{
    bool result = false;
    switch (reg_addr)
    {
    case GYRO_CONFIG0:
        result = IMU_sim_set_gyro_range(*data);
        result &= IMU_sim_set_gyro_freq(*data);
        break;
    case ACCEL_CONFIG0:
        result = IMU_sim_set_accel_range(*data);
        result &= IMU_sim_set_accel_freq(*data);
        break;
    case PWR_MGMT0:
        result = IMU_sim_set_accel_LP_clk(*data);
        result &= IMU_sim_set_idle(*data);
        result &= IMU_sim_set_gyro_mode(*data);
        result &= IMU_sim_set_accel_mode(*data);
        break;
    default:
        break;
    }
    return result;
}

bool IMU_sim_set_gyro_range(uint8_t gyro_range_reg_val)
{
    uint16_t range_val = 0;
    imu.reg.gyro_config0 = gyro_range_reg_val;
    range_val = gyro_range_reg_val & GYRO_UI_FS_SEL_MASK;
    range_val >> GYRO_UI_FS_SEL_POS;
    switch (range_val)
    {
    case GYRO_UI_FS_SEL_2000DPS:
        imu.gyro_range = 2000;
        break;
    case GYRO_UI_FS_SEL_1000DPS:
        imu.gyro_range = 1000;
        break;
    case GYRO_UI_FS_SEL_500DPS:
        imu.gyro_range = 500;
        break;
    case GYRO_UI_FS_SEL_250DPS:
        imu.gyro_range = 250;
        break;
    default:
        return false;
    }
    return true;
}

bool IMU_sim_set_accel_range(uint8_t accel_range_reg_val)
{
    uint16_t range_val = 0;
    imu.reg.accel_config0 = accel_range_reg_val;
    range_val = accel_range_reg_val & ACCEL_UI_FS_SEL_MASK;
    range_val >> ACCEL_UI_FS_SEL_POS;
    switch (range_val)
    {
    case ACCEL_UI_FS_SEL_2G:
        imu.accel_range = 2;
        break;
    case ACCEL_UI_FS_SEL_4G:
        imu.accel_range = 4;
        break;
    case ACCEL_UI_FS_SEL_8G:
        imu.accel_range = 8;
        break;
    case ACCEL_UI_FS_SEL_16G:
        imu.accel_range = 16;
        break;
    default:
        return false;
    }
    return true;
}

bool IMU_sim_set_gyro_freq(uint8_t gyro_freq_reg_val)
{
    uint16_t freq_val = 0;
    imu.reg.gyro_config0 = gyro_freq_reg_val;
    freq_val = gyro_freq_reg_val & GYRO_ODR_MASK;
    switch (freq_val)
    {
    case GYRO_ODR_1600HZ:
        imu.gyro_freq = 1600;
        break;
    case GYRO_ODR_800HZ:
        imu.gyro_freq = 800;
        break;
    case GYRO_ODR_400HZ:
        imu.gyro_freq = 400;
        break;
    case GYRO_ODR_200HZ:
        imu.gyro_freq = 200;
        break;
    case GYRO_ODR_100HZ:
        imu.gyro_freq = 100;
        break;
    case GYRO_ODR_50HZ:
        imu.gyro_freq = 50;
        break;
    case GYRO_ODR_25HZ:
        imu.gyro_freq = 25;
        break;
    case GYRO_ODR_12_5HZ:
        imu.gyro_freq = 12;
        break;
    default:
        return false;
    }
    return true;
}

bool IMU_sim_set_accel_freq(uint8_t accel_freq_reg_val)
{
    uint16_t freq_val = 0;
    imu.reg.accel_config0 = accel_freq_reg_val;
    freq_val = accel_freq_reg_val & ACCEL_ODR_MASK;

    switch (freq_val)
    {
    case ACCEL_ODR_1600HZ:
        imu.accel_freq = 1600;
        break;
    case ACCEL_ODR_800HZ:
        imu.accel_freq = 800;
        break;
    case ACCEL_ODR_400HZ:
        imu.accel_freq = 400;
        break;
    case ACCEL_ODR_200HZ:
        imu.accel_freq = 200;
        break;
    case ACCEL_ODR_100HZ:
        imu.accel_freq = 100;
        break;
    case ACCEL_ODR_50HZ:
        imu.accel_freq = 50;
        break;
    case ACCEL_ODR_25HZ:
        imu.accel_freq = 25;
        break;
    case ACCEL_ODR_12_5HZ:
        imu.accel_freq = 12;
        break;
    case ACCEL_ODR_6_25HZ:
        imu.accel_freq = 6;
        break;
    case ACCEL_ODR_3_125HZ:
        imu.accel_freq = 3;
        break;
    case ACCEL_ODR_1_5625HZ:
        imu.accel_freq = 2;
        break;
    default:
        return false;
    }

    return true;
}

bool IMU_sim_set_gyro_mode(uint8_t gyro_mode_reg_val)
{
    uint16_t gyro_mode_val = 0;
    imu.reg.pwr_mgmt0 = gyro_mode_reg_val;
    gyro_mode_val = (gyro_mode_reg_val & GYRO_MODE_MASK) >> GYRO_MODE_POS;
    switch (gyro_mode_val)
    {
    case GYRO_MODE_OFF:
        imu.gyro_on = false;
        break;
    case GYRO_MODE_STANDBY:
        imu.gyro_on = false;
        break;
    case GYRO_MODE_LOW_NOISE:
        imu.gyro_on = true;
        break;
    default:
        return false;
    }
    return true;
}

bool IMU_sim_set_accel_mode(uint8_t accel_mode_reg_val)
{
    uint16_t accel_mode_val = 0;
    imu.reg.pwr_mgmt0 = accel_mode_reg_val;
    accel_mode_val = accel_mode_reg_val & ACCEL_MODE_MASK;

    switch (accel_mode_val)
    {
    case ACCEL_MODE_OFF:
        imu.accel_on = false;
        break;
    case ACCEL_MODE_LOW_POWER:
        imu.accel_on = true;
        break;
    case ACCEL_MODE_LOW_NOISE:
        imu.accel_on = true;
        break;
    default:
        return false;
    }
    return true;
}

bool IMU_sim_set_accel_LP_clk(uint8_t reg_val)
{
    imu.reg.pwr_mgmt0 = reg_val;
    return true;
}

bool IMU_sim_set_idle(uint8_t reg_val)
{
    imu.reg.pwr_mgmt0 = reg_val;
    return true;
}

void IMU_sim_set_data_ready()
{
    imu.data_ready = true;
    imu.reg.int_status_drdy = 0x01;
}

void IMU_sim_clear_is_data_ready()
{
    imu.data_ready = false;
    imu.reg.int_status_drdy = 0x00;
    return;
}

bool create_resp_pipe()
{
    if (mkfifo(PIPE_RESP_NAME, 0666) == -1)
    {
        if (errno != EEXIST)
        {
            perror("mkfifo response pipe");
            return false;
        }
    }
    return true;
}

void close_resp_pipe()
{
    unlink(PIPE_RESP_NAME);
}

void process_command(const char *command, char *response)
{
    if (strncmp(command, "M,write,", 8) == 0)
    {
        unsigned int reg_addr, reg_val;
        sscanf(command + 8, "%02X,%02X", &reg_addr, &reg_val);
        // printf("Processing write command: RA=%02X, reg_val=%02X\n", reg_addr, reg_val);

        uint8_t reg_addr_u8 = (uint8_t)reg_addr;
        uint8_t reg_val_u8 = (uint8_t)reg_val;

        if (handle_write_register(reg_addr, &reg_val_u8))
        {
            snprintf(response, RESPONSE_BUF_SIZE, "S,OK");
        }
        else{
            snprintf(response, RESPONSE_BUF_SIZE, "S,ERR");
        }
    }
    else if (strncmp(command, "M,read,", 7) == 0)
    {
        unsigned int reg_addr;
        sscanf(command + 7, "%02X", &reg_addr);
        // printf("Processing read command: RA=%02X\n", reg_addr);

        uint8_t reg_addr_u8 = (uint8_t)reg_addr;
        uint8_t reg_val;
        if (handle_read_register(reg_addr, &reg_val))
        {
            snprintf(response, RESPONSE_BUF_SIZE, "S,%02X", reg_val);
        }
        else
        {
            snprintf(response, RESPONSE_BUF_SIZE, "S,ERR");
        }
    }
    else
    {
        snprintf(response, RESPONSE_BUF_SIZE, "S,ERR");
    }
    send_response(response);
}

void read_and_answer_command(){
    char buffer[RESPONSE_BUF_SIZE];
    char response[RESPONSE_BUF_SIZE];
    int fd = open(PIPE_CMD_NAME, O_RDONLY);

    while (fd == -1)
    {
        fd = open(PIPE_CMD_NAME, O_RDONLY);
        usleep(100000); // wait 100ms
    }
    ssize_t bytesRead = read(fd, buffer, sizeof(buffer) - 1);
    if (bytesRead > 0)
    {
        buffer[bytesRead] = '\0'; // terminate the string
        printf("Received command: %s\n", buffer);
        process_command(buffer, response);
    }
}

bool send_response(char response[])
{
    int fd = open(PIPE_RESP_NAME, O_WRONLY);
    if (fd == -1)
    {
        perror("open resp pipe");
        return false;
    }
    write(fd, response, strlen(response));
    close(fd);
    printf("Sent response: %s\n", response);
    return true;
}
