#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include "i2c_emul.h"
#include "i2c.h"
#include "ICM_42670_P_driver.h"

static bool get_response(char *response, size_t size, uint8_t *ret_val);
static bool process_response(char* response, uint8_t* ret_val);

bool i2c_init(uint8_t sda, uint8_t scl, uint8_t hz)
{
    return true;
}

bool i2c_write_data(const uint8_t reg_addr, uint8_t* send_reg_val)
{
    uint8_t command_size = 50;
    char write_command[command_size];
    char response[RESPONSE_BUF_SIZE];
    snprintf(write_command, sizeof(write_command), "M,write,%02X,%02X", reg_addr, *send_reg_val);
    printf("Sending write command: %s\n", write_command);
    int fd_cmd = open(PIPE_CMD_NAME, O_WRONLY);
    // if (fd_cmd == -1)
    // {
    //     perror("open");
    //     return false;
    // }
    write(fd_cmd, write_command, strlen(write_command) + 1);
    get_response(response, RESPONSE_BUF_SIZE, NULL);
    close(fd_cmd);
    return true;
}

bool i2c_read_data(const uint8_t reg_addr, uint8_t* rcv_reg_val)
{
    uint8_t command_size = 50;
    char read_command[command_size];
    char response[RESPONSE_BUF_SIZE];
    snprintf(read_command, sizeof(read_command), "M,read,%02X", reg_addr);

    printf("Sending read command: %s\n", read_command);
    int fd = open(PIPE_CMD_NAME, O_WRONLY);
    if (fd == -1)
    {
        perror("open cmd pipe");
        while (fd == -1)
        {
            fd = open(PIPE_CMD_NAME, O_WRONLY);
            usleep(100000); // wait 100ms
        }
        return false;
    }
    write(fd, read_command, strlen(read_command) + 1);
    get_response(response, RESPONSE_BUF_SIZE, rcv_reg_val);
    close(fd);
    return true;
}

static bool process_response(char *response, uint8_t *rcv_reg_val)
{
    if (strncmp(response, SLAVE_OK, 4) == 0)
    {
        // printf("Operation successful\n");
    }
    else if (strncmp(response, SLAVE_ERR, 5) == 0)
    {
        // printf("Operation failed\n");
    }
    else if (response[0] == 'S' && response[1] == ',')
    {
        unsigned int reg_val;
        sscanf(response + 2, "%02X", &reg_val);
        *rcv_reg_val = (uint8_t)reg_val;
        // printf("Received data: %02X\n", *rcv_reg_val);
    }
}

static bool get_response(char response[], size_t size_response, uint8_t *rcv_reg_val)
{
    int fd = open(PIPE_RESP_NAME, O_RDONLY);
    if (fd == -1)
    {
        perror("open response pipe");
        // while (fd == -1)
        // {
        //     fd = open(PIPE_RESP_NAME, O_RDONLY);
        //     usleep(100000); // wait 100ms
        // }
        return false;
    }

    size_t bytesRead = read(fd, response, size_response - 1);
    if (bytesRead > 0)
    {
        response[bytesRead] = '\0'; // terminate the string
        printf("Received response: %s\n", response);
        process_response(response, rcv_reg_val);
    }

    close(fd);
    return true;
}

bool create_cmd_pipe()
{
    if (mkfifo(PIPE_CMD_NAME, 0666) == -1)
    {
        if (errno != EEXIST)
        {
            perror("mkfifo command pipe");
            return false;
        }
    }
    return true;
}

void unlink_cmd_pipe()
{
    unlink(PIPE_CMD_NAME);
}