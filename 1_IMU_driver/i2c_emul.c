#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <poll.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include "i2c_emul.h"
#include "i2c.h"
#include "ICM_42670_P_driver.h"

static char response[I2C_BUF_SIZE];
static bool is_cmd_to_repeat = false;

static bool send_command(const uint8_t reg_addr, uint8_t* send_reg_val);
static bool get_and_process_response(uint8_t* rcv_reg_val);
static bool process_response(uint8_t* rcv_reg_val);

static int fd;
void ignore_sigpipe() {
    struct sigaction sa;
    sa.sa_handler = SIG_IGN;
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGPIPE, &sa, NULL);
}

bool i2c_init(uint8_t sda, uint8_t scl, uint8_t hz)
{
    create_pipes();
    //ignore SIGPIPE to not bloking the application at communication timeout
    ignore_sigpipe();
    
    return true;
}

bool i2c_write_data(const uint8_t reg_addr, uint8_t* send_reg_val)
{
    uint8_t* rcv_reg_val = NULL;

    do{
        send_command(reg_addr, send_reg_val);
        get_and_process_response(rcv_reg_val);
    } while (is_cmd_to_repeat);
    
    return true;
}

bool i2c_read_data(const uint8_t reg_addr, uint8_t* rcv_reg_val)
{
    uint8_t* send_reg_val = NULL;

    do{
        send_command(reg_addr, send_reg_val);
        get_and_process_response(rcv_reg_val);
    } while (is_cmd_to_repeat);

    return true;
}

static bool send_command(const uint8_t reg_addr, uint8_t* send_reg_val) {
    char write_command[I2C_BUF_SIZE];
    if (send_reg_val == NULL) {
        snprintf(write_command, sizeof(write_command), "M,read,%02X", reg_addr);
#if (I2C_EMUL_SHOW_COMMUNICATION == 1)
        printf("Sending read command: %s", write_command);
#endif
    }
    else {
        snprintf(write_command, sizeof(write_command), "M,write,%02X,%02X", reg_addr, *send_reg_val);
#if (I2C_EMUL_SHOW_COMMUNICATION == 1)
        printf("Write command sent: %s", write_command);
#endif
    }
    int fd_cmd = open(PIPE_CMD_NAME, O_WRONLY);
    if (fd_cmd == -1)
    {
        perror("open cmd pipe");
        return false;
    }
    ssize_t bytes_written = write(fd_cmd, write_command, strlen(write_command) + 1);
    if (bytes_written == -1) {
        if (errno == EPIPE) {
#if (I2C_EMUL_SHOW_COMMUNICATION == 1)
            fprintf(stderr, "Broken pipe: Reader process is not available\n");
#endif
        }
        else {
            perror("write");
        }
    }
#if (I2C_EMUL_SHOW_COMMUNICATION == 1)
    printf(" ...ok\n");
#endif
    close(fd_cmd);
    return true;
}


static bool get_and_process_response(uint8_t* rcv_reg_val)
{
    // Read from the internal pipe
    int fd = open(PIPE_RESP_NAME, O_RDONLY | O_NONBLOCK);
    if (fd == -1) {
        perror("open response pipe");
        return false;
    }

    struct pollfd fds;
    fds.fd = fd;
    fds.events = POLLIN;

    // Poll for data with a timeout
    int ret = poll(&fds, 1, RESPONSE_TIMEOUT_MS);
    if (ret == -1) {
        perror("poll error");
        is_cmd_to_repeat = true;
        close(fd);
        return false;
    }
    else if (ret == 0) {
#if (I2C_EMUL_SHOW_COMMUNICATION == 1)
        fprintf(stderr, "poll timeout\n");
#endif
        is_cmd_to_repeat = true;
        close(fd);
        return false;
    }
    ssize_t bytesRead = read(fd, response, I2C_BUF_SIZE - 1);
    if (bytesRead > 0) {
        response[bytesRead] = '\0'; // Terminate the string
#if (I2C_EMUL_SHOW_COMMUNICATION == 1)
        printf("Received response: %s\n", response);
#endif
        process_response(rcv_reg_val);
    }
    else if (bytesRead == -1) {
        is_cmd_to_repeat = true;
#if (I2C_EMUL_SHOW_COMMUNICATION == 1)
        perror("read error");
#endif
    }

    close(fd);
    return true;
}

static bool process_response(uint8_t* rcv_reg_val)
{
    if (strncmp(response, SLAVE_OK, 4) == 0)
    {
        is_cmd_to_repeat = false;
    }
    else if (strncmp(response, SLAVE_ERR, 5) == 0)
    {
        is_cmd_to_repeat = true;
    }
    else if (response[0] == 'S' && response[1] == ',' && rcv_reg_val != NULL)
    {
        unsigned int reg_val;
        sscanf(response + 2, "%02X", &reg_val);
        *rcv_reg_val = (uint8_t)reg_val;
        is_cmd_to_repeat = false;
    }
}

void create_pipes()
{
    if (mkfifo(PIPE_CMD_NAME, 0666) == -1)
    {
        if (errno != EEXIST)
        {
            perror("mkfifo response pipe");
        }
    }
    if (mkfifo(PIPE_RESP_NAME, 0666) == -1)
    {
        if (errno != EEXIST)
        {
            perror("mkfifo response pipe");
        }
    }
}

void close_pipes()
{
    unlink(PIPE_CMD_NAME);
    unlink(PIPE_RESP_NAME);
}